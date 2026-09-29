#include "GNSS.h"
#include "RoadSim.h"
#ifdef VIETHUD_P4
#include "net/PhoneGpsBle.h"
#endif
#include "core/AppConfig.h"
#include "core/SharedState.h"
#include "pincfg.h"
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <string.h> // memcpy (UBX MON-VER)
#include <stdlib.h> // atoi (GSV counts)

// Bench drive simulator state (see gnssSimStart in GNSS.h).
static volatile bool gSimActive = false;
static float gSimLat = 0, gSimLon = 0, gSimHeadingDeg = 0, gSimSpeedKmh = 0;
static uint32_t gSimEndMs = 0, gSimLastStepMs = 0, gSimFixSeq = 0;
// Waypoint mode (gnssSimRoute): follow a polyline instead of a straight line.
static const int kSimMaxWp = 48;
static float gSimWpLat[kSimMaxWp], gSimWpLon[kSimMaxWp];
static int gSimWpN = 0, gSimWpNext = 0;
void gnssSimRoute(const float *lat, const float *lon, int n, float speedKmh) {
    if (n < 2) return;
    if (n > kSimMaxWp) n = kSimMaxWp;
    for (int i = 0; i < n; i++) { gSimWpLat[i] = lat[i]; gSimWpLon[i] = lon[i]; }
    gSimWpN = n;
    gSimWpNext = 1;
    gSimLat = lat[0];
    gSimLon = lon[0];
    gSimSpeedKmh = speedKmh;
    float dy = (lat[1] - lat[0]) * 110540.0f, dx = (lon[1] - lon[0]) * 111320.0f * cosf(lat[0] * 0.0174533f);
    gSimHeadingDeg = fmodf(atan2f(dx, dy) * 57.29578f + 360.0f, 360.0f);
    gSimLastStepMs = millis();
    gSimEndMs = gSimLastStepMs + 3600000UL; // ends at the last waypoint
    gSimActive = true;
    Serial.printf("[gnss] route simulation: %d waypoints at %.0f km/h\n", n, (double)speedKmh);
}
void gnssSimStart(float lat, float lon, float headingDeg, float speedKmh, float seconds) {
    if (seconds <= 0) {
        gSimActive = false;
        Serial.println("[gnss] drive simulation stopped");
        return;
    }
    gSimWpN = 0;
    gSimLat = lat;
    gSimLon = lon;
    gSimHeadingDeg = headingDeg;
    gSimSpeedKmh = speedKmh;
    gSimLastStepMs = millis();
    gSimEndMs = gSimLastStepMs + (uint32_t)(seconds * 1000.0f);
    gSimActive = true;
    Serial.printf("[gnss] drive simulation: %.6f,%.6f hdg %.0f %.0f km/h for %.0f s\n", (double)lat, (double)lon,
                  (double)headingDeg, (double)speedKmh, (double)seconds);
}

// Confirmed on real hardware 2026-09-15 by sweeping candidate bauds and
// dumping raw bytes: this specific M10N breakout is NOT the common
// 9600-by-default kind — 9600 decoded as binary garbage (proved it wasn't
// a wiring fault), 38400 decoded clean, valid NMEA immediately (12 sats,
// real fix). Don't assume 9600 for a "default" M10N breakout again; verify
// per-board like this was.
static const uint32_t kGnssBaud = 38400;

// Spec section 5.3: raw GNSS speed must be filtered (median, then EMA/
// low-pass) before it drives the audio-gate hysteresis, so sensor noise
// can't itself cause the 60 km/h gate to chatter — the hysteresis band in
// AppConfig only protects against genuine speed oscillation, not noise on
// top of it.
class SpeedFilter {
public:
    float push(float raw) {
        hist[histIdx] = raw;
        histIdx = (histIdx + 1) % 3;
        if (histFill < 3) histFill++;

        float median = raw;
        if (histFill == 3) {
            float a = hist[0], b = hist[1], c = hist[2];
            median = fmaxf(fminf(a, b), fminf(fmaxf(a, b), c)); // median of 3
        }

        // Reads cfg directly (no lock) — same conscious simplification
        // AppConfig.h documents for SimTask: every field is an
        // independently-aligned float, so a cross-task read during a
        // Settings write sees only one field's old-or-new value, never a
        // torn one, and there's no multi-field invariant here to protect.
        float alpha = cfg.gnssSpeedFilterAlpha;
        ema = emaInit ? (alpha * median + (1.0f - alpha) * ema) : median;
        emaInit = true;
        return ema;
    }

    void reset() { histFill = 0; histIdx = 0; emaInit = false; }

private:
    float hist[3] = {0, 0, 0};
    uint8_t histIdx = 0, histFill = 0;
    float ema = 0;
    bool emaInit = false;
};

static TinyGPSPlus gps;

// Sunrise/sunset for the Dashboard clock icon (spec section 15.2: "no
// light sensor" — use GNSS date+time+lat+lon instead). Standard solar
// declination approximation (Cooper's equation) + hour-angle formula, no
// equation-of-time correction — good to within a few minutes near
// equinoxes, which is all an icon swap needs; not used for anything
// safety-related.
static bool computeDaytime(float latDeg, float lonDeg, int year, int month, int day, int utcHour, int utcMinute) {
    static const int kCumDays[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    int dayOfYear = kCumDays[month - 1] + day + ((leap && month > 2) ? 1 : 0);

    float declRad = (23.45f * (float)M_PI / 180.0f) * sinf(2.0f * (float)M_PI * (284.0f + dayOfYear) / 365.0f);
    float latRad = latDeg * (float)M_PI / 180.0f;

    float cosH = -tanf(latRad) * tanf(declRad);
    if (cosH <= -1.0f) return true;  // polar day: sun never sets
    if (cosH >= 1.0f) return false;  // polar night: sun never rises
    float hHours = acosf(cosH) * (180.0f / (float)M_PI) / 15.0f;

    float solarNoonUtc = 12.0f - lonDeg / 15.0f;
    float sunriseUtc = solarNoonUtc - hHours;
    float sunsetUtc = solarNoonUtc + hHours;
    float nowUtc = utcHour + utcMinute / 60.0f;

    return nowUtc >= sunriseUtc && nowUtc < sunsetUtc;
}

// ---------------------------------------------------------------------------
// u-blox UBX configuration (2026-09-26). Until now the module ran on its
// factory defaults (NMEA only, "portable" dynamics). At every boot we now:
//   * poll UBX-MON-VER (firmware + supported GNSS, logged for diagnosis);
//   * set the AUTOMOTIVE dynamic model (smoother, road-constrained fixes);
//   * enable AssistNow Autonomous (on-chip orbit prediction = A-GNSS without
//     internet; survives power-off only if the breakout has a backup supply);
//   * enable GPS + BeiDou (B1I) + Galileo (+ QZSS, SBAS) — see kSignals.
// Written to the RAM layer only: nothing persists in the module, so a power
// cycle always returns it to its known factory state.
// ---------------------------------------------------------------------------
struct UbxKv {
    uint32_t key;
    uint8_t size; // value bytes: 1, 2 or 4
    uint32_t val;
};

static void ubxSend(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len) {
    uint8_t hdr[6] = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6; i++) { a += hdr[i]; b += a; }
    for (uint16_t i = 0; i < len; i++) { a += pl[i]; b += a; }
    uint8_t ck[2] = {a, b};
    Serial2.write(hdr, 6);
    if (len) Serial2.write(pl, len);
    Serial2.write(ck, 2);
}

static void ubxValset(const UbxKv *kv, int n) {
    static uint8_t pl[4 + 16 * 8];
    int len = 0;
    pl[len++] = 0x00; // version
    pl[len++] = 0x01; // layers: RAM only
    pl[len++] = 0x00;
    pl[len++] = 0x00;
    for (int i = 0; i < n && len + 8 <= (int)sizeof(pl); i++) {
        for (int k = 0; k < 4; k++) pl[len++] = (uint8_t)(kv[i].key >> (8 * k));
        for (int k = 0; k < kv[i].size; k++) pl[len++] = (uint8_t)(kv[i].val >> (8 * k));
    }
    ubxSend(0x06, 0x8A, pl, len); // UBX-CFG-VALSET
}

// Minimal UBX frame receiver interleaved with the NMEA stream (0xB5 never
// occurs in NMEA ASCII, so UBX bytes are cleanly separable from TinyGPS input).
static volatile int8_t sValsetAck = 0; // 0 pending, 1 ACK, -1 NAK
static bool ubxFeed(uint8_t c) {
    static uint8_t st = 0, cls, id, ckA, ckB;
    static uint16_t len, got;
    static uint8_t buf[200];
    switch (st) {
    case 0: if (c == 0xB5) { st = 1; return true; } return false;
    case 1: if (c == 0x62) { st = 2; ckA = ckB = 0; return true; } st = 0; return false;
    case 2: cls = c; ckA += c; ckB += ckA; st = 3; return true;
    case 3: id = c; ckA += c; ckB += ckA; st = 4; return true;
    case 4: len = c; ckA += c; ckB += ckA; st = 5; return true;
    case 5:
        len |= (uint16_t)c << 8; ckA += c; ckB += ckA; got = 0;
        st = len ? 6 : 7;
        return true;
    case 6:
        if (got < sizeof(buf)) buf[got] = c;
        got++; ckA += c; ckB += ckA;
        if (got >= len) st = 7;
        return true;
    case 7: st = (c == ckA) ? 8 : 0; return true;
    case 8: {
        st = 0;
        if (c != ckB) return true;
        uint16_t n = len < sizeof(buf) ? len : sizeof(buf);
        if (cls == 0x05 && n >= 2 && buf[0] == 0x06 && buf[1] == 0x8A) {
            sValsetAck = (id == 0x01) ? 1 : -1; // ACK-ACK / ACK-NAK for CFG-VALSET
        } else if (cls == 0x0A && id == 0x04 && n >= 40) { // MON-VER
            char sw[31], hw[11];
            memcpy(sw, buf, 30); sw[30] = 0;
            memcpy(hw, buf + 30, 10); hw[10] = 0;
            for (char *q = sw; *q; q++) if (*q < 0x20 || *q > 0x7E) *q = '?';
            for (char *q = hw; *q; q++) if (*q < 0x20 || *q > 0x7E) *q = '?';
            Serial.printf("[gnss] module: sw=\"%s\" hw=\"%s\"\n", sw, hw);
            for (uint16_t o = 40; o + 30 <= n; o += 30) {
                char ext[31];
                memcpy(ext, buf + o, 30); ext[30] = 0;
                Serial.printf("[gnss]   %s\n", ext);
            }
        }
        return true;
    }
    }
    st = 0;
    return false;
}

// Runs the configuration steps; called every loop tick until done.
static void ubxConfigStep(uint32_t sinceBootMs) {
    static int step = 0;
    static uint32_t sentAt = 0;
    static const UbxKv kBase[] = {
        {0x20110021, 1, 4}, // CFG-NAVSPG-DYNMODEL = 4 (automotive)
        {0x10230001, 1, 1}, // CFG-ANA-USE_ANA = 1 (AssistNow Autonomous)
    };
    // GPS + BeiDou B1I + Galileo (+ QZSS, SBAS). Verified on this module
    // (PROTVER 34.10): adding GLONASS is REJECTED — the M10's single RF path
    // can't receive GLONASS L1 (1602 MHz) together with BeiDou B1I (1561 MHz).
    // For Vietnam B1I is the better pick: it includes BeiDou's GEO/IGSO
    // satellites that sit permanently over Asia (GLONASS adds little here).
    static const UbxKv kSignals[] = {
        {0x1031001f, 1, 1}, {0x10310001, 1, 1}, // GPS L1C/A
        {0x10310022, 1, 1}, {0x1031000d, 1, 1}, // BeiDou B1I
        {0x10310021, 1, 1}, {0x10310007, 1, 1}, // Galileo E1
        {0x10310025, 1, 0},                     // GLONASS off (see above)
        {0x10310024, 1, 1}, {0x10310012, 1, 1}, // QZSS L1C/A
        {0x10310020, 1, 1}, {0x10310005, 1, 1}, // SBAS L1C/A
    };
    auto waitAck = [&](const char *what) -> int { // 1 ack, -1 nak/timeout, 0 still waiting
        if (sValsetAck == 0 && sinceBootMs - sentAt < 1500) return 0;
        int r = sValsetAck == 1 ? 1 : -1;
        Serial.printf("[gnss] config %s: %s\n", what, r == 1 ? "OK" : (sValsetAck == -1 ? "REJECTED" : "no answer"));
        return r;
    };
    switch (step) {
    case 0:
        if (sinceBootMs < 800) return;
        ubxSend(0x0A, 0x04, nullptr, 0); // poll MON-VER
        sentAt = sinceBootMs;
        step = 1;
        return;
    case 1:
        if (sinceBootMs - sentAt < 400) return;
        sValsetAck = 0; ubxValset(kBase, 2); sentAt = sinceBootMs; step = 2;
        return;
    case 2:
        if (!waitAck("automotive + AssistNow Autonomous")) return;
        sValsetAck = 0; ubxValset(kSignals, sizeof(kSignals) / sizeof(kSignals[0])); sentAt = sinceBootMs; step = 3;
        return;
    case 3:
        if (!waitAck("GPS+BeiDou+Galileo+QZSS+SBAS")) return;
        step = 99;
        return;
    default:
        return;
    }
}

// Satellites IN VIEW per constellation (GSV field 3), for diagnostics — shows
// whether BeiDou/Galileo/GLONASS are actually being tracked.
static TinyGPSCustom gsvGps(gps, "GPGSV", 3), gsvGlo(gps, "GLGSV", 3), gsvGal(gps, "GAGSV", 3),
    gsvBds(gps, "GBGSV", 3), gsvQzs(gps, "GQGSV", 3);
static int gsvCount(TinyGPSCustom &c) { return c.isValid() ? atoi(c.value()) : 0; }

// File-scope (not function-local) so gnssMsSinceStationary() below can read
// it — see GNSS.h's comment on that function for what this tracks.
static uint32_t lastMovingMs = 0;

uint32_t gnssMsSinceStationary() { return millis() - lastMovingMs; }

static void gnssTaskFn(void *) {
    Serial2.begin(kGnssBaud, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);

    SpeedFilter speedFilter;
    const uint32_t taskStartMs = millis();
    esp_task_wdt_add(NULL);
    uint32_t lastFixMs = 0;
    uint32_t lastValidSentenceMs = 0;
    // Heading hold state (GNSS.h's kHeadingHoldMs comment) — lastKnownHeadingMs
    // == 0 means "nothing to hold yet", same sentinel style as lastFixMs.
    float lastKnownHeadingDeg = 0;
    uint32_t lastKnownHeadingMs = 0;
    uint32_t lastPassedChecksum = 0;
    uint32_t lastDebugMs = 0;

    // "No fix yet" and "not receiving anything" used to both show as FAULT
    // — misleading, since the first one is the normal state while cold-
    // starting/indoors and the second is an actual wiring/power/baud
    // problem. linkAlive tracks the second, separately from fix.
    const uint32_t kLinkTimeoutMs = 5000; // generous vs. NMEA's ~1s sentence burst cadence

    for (;;) {
        while (Serial2.available()) {
            uint8_t c = (uint8_t)Serial2.read();
            if (!ubxFeed(c)) gps.encode((char)c); // UBX replies are consumed here, NMEA goes to TinyGPS
        }
        ubxConfigStep(millis() - taskStartMs);
#ifdef VIETHUD_P4
        // Phone GPS over BLE (net/PhoneGpsBle.h): same parser, but only while
        // the UART module has been silent for 5 s — the module always wins,
        // and the two NMEA streams are never interleaved in one parser.
        static uint32_t sUartOkMs = 0, sUartPassed = 0, sPhoneOkMs = 0;
        if (gps.passedChecksum() != sUartPassed) { // counted before the phone feed below
            sUartPassed = gps.passedChecksum();
            sUartOkMs = millis();
        }
        {
            uint8_t pb[256];
            size_t n;
            bool uartSilent = sUartOkMs == 0 || millis() - sUartOkMs > 5000;
            while ((n = phoneGpsRead(pb, sizeof(pb))) > 0) {
                if (!uartSilent) continue; // drained and dropped
                uint32_t before = gps.passedChecksum();
                for (size_t i = 0; i < n; i++) gps.encode((char)pb[i]);
                if (gps.passedChecksum() != before) sPhoneOkMs = millis();
            }
            sUartPassed = gps.passedChecksum(); // phone sentences are not UART activity
        }
        const bool fromPhone = sPhoneOkMs != 0 && millis() - sPhoneOkMs < 3000;
#endif

        uint32_t passedChecksum = gps.passedChecksum();
        if (passedChecksum != lastPassedChecksum) {
            lastPassedChecksum = passedChecksum;
            lastValidSentenceMs = millis();
        }
        bool linkAlive = lastValidSentenceMs != 0 && (millis() - lastValidSentenceMs) < kLinkTimeoutMs;

        bool freshFix = gps.location.isValid() && gps.location.isUpdated();
        static uint32_t sFixSeq = 0;
        if (freshFix) {
            lastFixMs = millis();
            sFixSeq++;
        }
        // GNSS considered "lost" if no fresh fix for this long — spec
        // section 23: never hold/assume a stale speed once lost, disable
        // the audio gate, show fault. Tunable from Settings > Sensors.
        uint32_t fixTimeoutMs = (uint32_t)(cfg.gnssFixTimeoutS * 1000.0f);
        bool haveRecentFix = gps.location.isValid() && (millis() - lastFixMs) < fixTimeoutMs;

        GnssSnapshot snap;
        snap.fix = haveRecentFix;
        snap.linkAlive = linkAlive;
#ifdef VIETHUD_P4
        snap.fromPhone = fromPhone;
#endif
        if (haveRecentFix && gps.speed.isValid()) {
            // Calibration applied here, before the filter, so both
            // rawSpeedKmh and the filtered egoSpeedKmh reflect the corrected
            // value (AppConfig.h's gnssSpeedCalibrationPct comment) — no
            // separate "true" vs "displayed" speed anywhere downstream.
            float measuredKmh = (float)gps.speed.kmph();
            snap.rawSpeedKmh = measuredKmh * (1.0f + cfg.gnssSpeedCalibrationPct / 100.0f);
            snap.egoSpeedKmh = speedFilter.push(snap.rawSpeedKmh);
        } else {
            speedFilter.reset();
            snap.rawSpeedKmh = 0;
            snap.egoSpeedKmh = 0; // consumers must gate on .fix, never trust this while !fix
        }
        snap.satCount = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
        snap.hdop = gps.hdop.isValid() ? (float)gps.hdop.hdop() : 0.0f;
        snap.fixSeq = sFixSeq;

        // Position, independent of time/date validity — map/SpeedLimitMap.cpp
        // (added 2026-09-15) needs lat/lon whenever there's a fix at all, not
        // only once the clock's own separate fields are also valid. Moved out
        // of the timeValid block below, which used to bundle lon/lat in with
        // it purely because the Dashboard clock (the only consumer before
        // now) happened to need both; that was never a real dependency
        // between the two. No behavior change for the clock: every case
        // where timeValid was true already had location.isValid() as one of
        // its ANDed conditions, so lon/lat were always set in that case
        // either way — this only ADDS the case of "fix valid, date/time not
        // yet" where they're now populated too.
        if (haveRecentFix && gps.location.isValid()) {
            snap.lonDeg = (float)gps.location.lng();
            snap.latDeg = (float)gps.location.lat();
        }
        if (haveRecentFix && gps.altitude.isValid()) {
            snap.altitudeM = (float)gps.altitude.meters();
            snap.altitudeValid = true;
        }

        // Course over ground (spec architecture note: "M10N xác định vị trí,
        // tốc độ, hướng" — heading was the missing piece). A GPS module
        // derives course from the motion between fixes, not a compass, so
        // it's meaningless/noisy near-stationary — gated on a minimum speed,
        // same "don't trust a noisy low-signal reading" reasoning as the
        // speed filter's own median+EMA above. kGnssMotionThresholdKmh (see
        // GNSS.h) is this project's one shared constant for "real motion vs.
        // GPS noise floor" — also reused just below for the auto-dim gate.
        bool liveHeadingValid = haveRecentFix && gps.course.isValid() && snap.rawSpeedKmh > kGnssMotionThresholdKmh;
        if (liveHeadingValid) {
            snap.headingDeg = (float)gps.course.deg();
            snap.headingValid = true;
            snap.headingPredicted = false;
            lastKnownHeadingDeg = snap.headingDeg;
            lastKnownHeadingMs = millis();
        } else if (haveRecentFix && lastKnownHeadingMs != 0 &&
                   (millis() - lastKnownHeadingMs) < kHeadingHoldMs) {
            // Bridge a brief GPS-course gap (car slowed below
            // kGnssMotionThresholdKmh, or the module's own course briefly
            // glitched) by holding the last known heading — GNSS.h's
            // kHeadingHoldMs comment has the full "dự đoán hướng di chuyển
            // của xe" reasoning. Requires a still-current FIX, not just a
            // recent one: a real position jump (module re-acquiring after a
            // gap) must never carry a stale direction forward.
            snap.headingDeg = lastKnownHeadingDeg;
            snap.headingValid = true;
            snap.headingPredicted = true;
        } else {
            snap.headingValid = false;
            snap.headingPredicted = false;
        }
        // A real fix loss invalidates any held heading immediately — once
        // GPS comes back (e.g. after a tunnel), the car could easily be
        // pointed a different way than before the gap, so the next fix must
        // re-earn a live heading rather than resume holding the old one.
        if (!haveRecentFix) lastKnownHeadingMs = 0;

        // Dashboard clock + day/night icon: only trust GPS time/date
        // alongside a real position fix, since both the local-time
        // estimate and the sunrise/sunset calc need lat/lon anyway — same
        // "don't show it if we don't trust it" rule as speed above.
        snap.timeValid = haveRecentFix && gps.time.isValid() && gps.date.isValid() && gps.location.isValid();
        if (snap.timeValid) {
            snap.utcHour = gps.time.hour();
            snap.utcMinute = gps.time.minute();
            snap.daytime = computeDaytime(snap.latDeg, snap.lonDeg, gps.date.year(), gps.date.month(),
                                           gps.date.day(), snap.utcHour, snap.utcMinute);
        }

        // Auto-dim's "vehicle stationary" gate (Dashboard.cpp's
        // burnInTimerCb) — see GNSS.h's gnssMsSinceStationary() comment.
        // !haveRecentFix counts as "moving" (reset the timer) — never assume
        // parked just because the fix dropped.
        if (!haveRecentFix || snap.egoSpeedKmh > kGnssMotionThresholdKmh) lastMovingMs = millis();

        if (gSimActive) {
            uint32_t nowS = millis();
            if ((int32_t)(nowS - gSimEndMs) >= 0) {
                gSimActive = false;
                Serial.println("[gnss] drive simulation finished");
            } else if (nowS - gSimLastStepMs >= 200) {
                float dt = (nowS - gSimLastStepMs) / 1000.0f;
                gSimLastStepMs = nowS;
                float dM = gSimSpeedKmh / 3.6f * dt;
                if (gSimWpN > 0) {
                    // consume dM along the polyline
                    while (dM > 0 && gSimWpNext < gSimWpN) {
                        float kx = 111320.0f * cosf(gSimLat * 0.0174533f);
                        float dy = (gSimWpLat[gSimWpNext] - gSimLat) * 110540.0f;
                        float dx = (gSimWpLon[gSimWpNext] - gSimLon) * kx;
                        float L = sqrtf(dx * dx + dy * dy);
                        if (L > 0.01f) gSimHeadingDeg = fmodf(atan2f(dx, dy) * 57.29578f + 360.0f, 360.0f);
                        if (L <= dM) {
                            gSimLat = gSimWpLat[gSimWpNext];
                            gSimLon = gSimWpLon[gSimWpNext];
                            gSimWpNext++;
                            dM -= L;
                        } else {
                            gSimLat += dy * (dM / L) / 110540.0f;
                            gSimLon += dx * (dM / L) / kx;
                            dM = 0;
                        }
                    }
                    if (gSimWpNext >= gSimWpN) {
                        gSimEndMs = nowS; // arrived: stop on the next pass
                        gSimWpN = 0;
                    }
                } else {
                    float hr = gSimHeadingDeg * 0.0174533f;
                    gSimLat += dM * cosf(hr) / 110540.0f;
                    gSimLon += dM * sinf(hr) / (111320.0f * cosf(gSimLat * 0.0174533f));
                }
                gSimFixSeq++;
            }
            if (gSimActive) {
                snap.fix = true;
                snap.linkAlive = true;
                snap.satCount = 14;
                snap.latDeg = gSimLat;
                snap.lonDeg = gSimLon;
                snap.egoSpeedKmh = snap.rawSpeedKmh = gSimSpeedKmh;
                snap.headingDeg = gSimHeadingDeg;
                snap.headingValid = true;
                snap.headingPredicted = false;
                snap.hdop = 0.9f;
                snap.altitudeValid = false;
                snap.fixSeq = 0x80000000u | gSimFixSeq;
            }
        }
        // Road-following simulator (gnss/RoadSim.h): serial "R lat lon" starts
        // it anywhere; with VIETHUD_ROADSIM_AUTOSTART (P4 bench board, no
        // GNSS fitted yet) it also starts by itself when no valid NMEA
        // sentence has arrived 8 s after boot. Yields once a real fix appears.
        {
#ifdef VIETHUD_ROADSIM_AUTOSTART
            // Bench/demo boards only (env:viethud_p4). NEVER in the car build:
            // an unplugged GPS there must read as "no GPS", not as a drive.
            static bool sAutoTried = false;
            if (!sAutoTried && !gSimActive && millis() > 8000 && gps.passedChecksum() == 0) {
                sAutoTried = true;
                Serial.println("[gnss] no GNSS module detected -> road simulation (real roads from the speed map)");
                roadSimRequestStart(kRoadSimDefaultLat, kRoadSimDefaultLon, 1.0f);
            }
#endif
            if (haveRecentFix && gps.passedChecksum() > 0 && roadSimActive()) {
                Serial.println("[gnss] real GNSS fix -> road simulation off");
                roadSimRequestStop();
            }
            float la, lo, hd, v;
            if (!gSimActive && roadSimStep(&la, &lo, &hd, &v)) {
                gSimFixSeq++;
                snap.fix = true;
                snap.linkAlive = true;
                snap.satCount = 14;
                snap.latDeg = la;
                snap.lonDeg = lo;
                snap.egoSpeedKmh = snap.rawSpeedKmh = v;
                snap.headingDeg = hd;
                snap.headingValid = true;
                snap.headingPredicted = false;
                snap.hdop = 0.9f;
                snap.altitudeValid = false;
                snap.fixSeq = 0x80000000u | gSimFixSeq;
            }
        }
        gnssPublish(snap);

        uint32_t now = millis();
        if (now - lastDebugMs > 3000) {
            lastDebugMs = now;
            Serial.printf("[gnss] fix=%d link=%d sats=%d speed=%.1fkm/h chars=%lu sentencesWithFix=%lu "
                          "failedChecksum=%lu\n",
                          snap.fix, snap.linkAlive, snap.satCount, (double)snap.egoSpeedKmh,
                          gps.charsProcessed(), gps.sentencesWithFix(), gps.failedChecksum());
            static uint32_t sLastViewLogMs = 0;
            if (now - sLastViewLogMs > 10000) {
                sLastViewLogMs = now;
                Serial.printf("[gnss] in view: GPS=%d BeiDou=%d Galileo=%d GLONASS=%d QZSS=%d\n", gsvCount(gsvGps),
                              gsvCount(gsvBds), gsvCount(gsvGal), gsvCount(gsvGlo), gsvCount(gsvQzs));
            }
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50)); // drain UART well inside NMEA's own ~1s sentence cadence
    }
}

// 3072, not the original 4096 — real-hardware measurement (2026-09-16 RAM
// audit): high-water mark never dropped below ~1944 bytes free of 4096, i.e. never used
// more than ~2152 bytes, leaving a comfortable ~920-byte (~30%) margin at
// 3072.
// 4096 since 2026-09-26: UBX configuration + MON-VER logging added to this task.
void gnssTaskStart() { xTaskCreatePinnedToCore(gnssTaskFn, "gnssTask", 4096, NULL, 2, NULL, 0); }

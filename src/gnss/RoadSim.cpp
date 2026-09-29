#include "RoadSim.h"
#include "map/SdCardManager.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <math.h>

// ---- small LRU cache of whole tiles (PSRAM), independent of the matcher's ----
static const int kTiles = 12;
static const int kMaxSegs = 2048;
struct SimTile {
    uint32_t id = 0xFFFFFFFFu;
    int n = 0;
    uint32_t lastUse = 0;
    RoadSegment *segs = nullptr;
};
static SimTile sTiles[kTiles];
static float sTileDeg = 0;
static uint32_t sUse = 0;

static const SimTile *tileGet(uint32_t id) {
    for (auto &t : sTiles)
        if (t.id == id) { t.lastUse = ++sUse; return &t; }
    SimTile *v = &sTiles[0];
    for (auto &t : sTiles)
        if (t.lastUse < v->lastUse) v = &t;
    if (!v->segs) v->segs = (RoadSegment *)heap_caps_malloc(sizeof(RoadSegment) * kMaxSegs, MALLOC_CAP_SPIRAM);
    if (!v->segs) return nullptr;
    v->id = id;
    v->n = 0;
    v->lastUse = ++sUse;
    TileIndexEntry e;
    if (sdMgrFindTileEntry(id, &e)) sdMgrReadTile(e, v->segs, kMaxSegs, &v->n); // missing tile = empty sea/no roads
    return v;
}

// Calls fn(seg) for every segment in the 3x3 tiles around (lat,lon).
template <class Fn> static void forNearby(int32_t latE7, int32_t lonE7, Fn fn) {
    uint32_t c = speedmapTileId(latE7 * 1e-7f, lonE7 * 1e-7f, sTileDeg);
    int32_t la = (int32_t)(c >> 16), lo = (int32_t)(c & 0xFFFF);
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++) {
            const SimTile *t = tileGet(((uint32_t)(la + dy) << 16) | ((uint32_t)(lo + dx) & 0xFFFF));
            if (!t) continue;
            for (int i = 0; i < t->n; i++) fn(t->segs[i]);
        }
}

// ---- geometry ----
static inline float kxAt(float latDeg) { return 111320.0f * cosf(latDeg * 0.0174533f); }
static float distM(int32_t aLat, int32_t aLon, int32_t bLat, int32_t bLon) {
    float dy = (bLat - aLat) * 1e-7f * 110540.0f, dx = (bLon - aLon) * 1e-7f * kxAt(aLat * 1e-7f);
    return sqrtf(dx * dx + dy * dy);
}
static float bearing(int32_t aLat, int32_t aLon, int32_t bLat, int32_t bLon) {
    float dy = (bLat - aLat) * 1e-7f * 110540.0f, dx = (bLon - aLon) * 1e-7f * kxAt(aLat * 1e-7f);
    return fmodf(atan2f(dx, dy) * 57.29578f + 360.0f, 360.0f);
}
static float angDiff(float a, float b) {
    float d = fabsf(fmodf(a - b + 540.0f, 360.0f) - 180.0f);
    return d;
}
static bool canFwd(const RoadSegment &s) { return s.direction != DIR_BACKWARD; }
static bool canBwd(const RoadSegment &s) { return s.direction != DIR_FORWARD; }

// A directed traversal of one segment.
struct Edge {
    RoadSegment s;
    bool fwd = true;
    bool valid = false;
    int32_t fromLat() const { return fwd ? s.startLatE7 : s.endLatE7; }
    int32_t fromLon() const { return fwd ? s.startLonE7 : s.endLonE7; }
    int32_t toLat() const { return fwd ? s.endLatE7 : s.startLatE7; }
    int32_t toLon() const { return fwd ? s.endLonE7 : s.startLonE7; }
    float len() const { return distM(fromLat(), fromLon(), toLat(), toLon()); }
    float hdg() const { return bearing(fromLat(), fromLon(), toLat(), toLon()); }
};

static float cruiseKmh(const RoadSegment &s) {
    if (s.speedLimitKmh > 0) return s.speedLimitKmh;
    switch (s.roadClass) {
    case 1: return 60;
    case 2: return 50;
    case 3: return 30;
    default: return 40;
    }
}
// Comfortable speed through a junction for a given heading change.
static float turnKmh(float turnDeg) {
    if (turnDeg > 120) return 12;
    if (turnDeg > 70) return 20;
    if (turnDeg > 35) return 32;
    return 999;
}

// ---- state (touched only by the GNSS task, except the request mailbox) ----
static volatile bool sReqStart = false, sReqStop = false, sActive = false;
static volatile float sReqLat = 0, sReqLon = 0, sReqFactor = 1.0f;
static Edge sCur, sNext;
static float sPosM = 0, sSpeedKmh = 0, sHeading = 0, sFactor = 1.0f;
static uint32_t sLastStepMs = 0, sLastLogMs = 0;

void roadSimRequestStart(float lat, float lon, float speedFactor) {
    sReqLat = lat;
    sReqLon = lon;
    sReqFactor = (speedFactor > 0.3f && speedFactor < 3.0f) ? speedFactor : 1.0f;
    sReqStart = true;
}
void roadSimRequestStop() { sReqStop = true; }
bool roadSimActive() { return sActive; }

// Choose how to leave the node at the end of `in`: mostly the straightest
// continuation, sometimes a random turn, preferring the same or a bigger road
// class; a dead end turns around. Returns an invalid Edge if there is truly
// nowhere to go (then the caller re-snaps).
static Edge chooseNext(const Edge &in) {
    const int32_t nLat = in.toLat(), nLon = in.toLon();
    const float inH = in.hdg();
    Edge cand[16];
    float turn[16];
    int n = 0;
    forNearby(nLat, nLon, [&](const RoadSegment &s) {
        if (n >= 16) return;
        for (int f = 0; f < 2; f++) {
            bool fwd = f == 0;
            int32_t aLat = fwd ? s.startLatE7 : s.endLatE7, aLon = fwd ? s.startLonE7 : s.endLonE7;
            if (aLat != nLat || aLon != nLon) continue;
            if (fwd ? !canFwd(s) : !canBwd(s)) continue;
            Edge e;
            e.s = s;
            e.fwd = fwd;
            e.valid = true;
            // skip going straight back the way we came
            if (e.toLat() == in.fromLat() && e.toLon() == in.fromLon()) continue;
            float t = angDiff(e.hdg(), inH);
            bool dup = false;
            for (int k = 0; k < n; k++)
                dup |= cand[k].toLat() == e.toLat() && cand[k].toLon() == e.toLon();
            if (dup || n >= 16) continue;
            cand[n] = e;
            turn[n++] = t;
        }
    });
    if (n == 0) { // dead end: U-turn if this road allows it
        Edge back = in;
        back.fwd = !in.fwd;
        if (in.fwd ? canBwd(in.s) : canFwd(in.s)) return back;
        return Edge();
    }
    // Score: turn angle, plus a penalty for dropping to a smaller road class
    // (classes: 1 major < 2 main < 3 small; 0 unclassified ~ small).
    int best = 0;
    float bestScore = 1e9f;
    int curCls = in.s.roadClass ? in.s.roadClass : 3;
    for (int k = 0; k < n; k++) {
        int cls = cand[k].s.roadClass ? cand[k].s.roadClass : 3;
        float score = turn[k] + (cls > curCls ? 45.0f * (cls - curCls) : 0.0f);
        if (score < bestScore) { bestScore = score, best = k; }
    }
    // 25% of the time at a real junction, take another sensible branch instead
    if (n > 1 && (esp_random() % 100) < 25) {
        int alt[16], m = 0;
        for (int k = 0; k < n; k++)
            if (k != best && turn[k] < 135 && (cand[k].s.roadClass ? cand[k].s.roadClass : 3) <= 3) alt[m++] = k;
        if (m) best = alt[esp_random() % m];
    }
    return cand[best];
}

// Snap to the nearest drivable segment around (lat,lon).
static bool snapStart(float lat, float lon) {
    SpeedMapMetadata md;
    if (!sdMgrIsAvailable() || !sdMgrGetMetadata(md) || md.tileSizeDeg <= 0) {
        Serial.println("[roadsim] no speed map on SD — cannot simulate on real roads");
        return false;
    }
    sTileDeg = md.tileSizeDeg;
    const int32_t pLat = (int32_t)(lat * 1e7f), pLon = (int32_t)(lon * 1e7f);
    float bestD = 1e9f;
    Edge best;
    forNearby(pLat, pLon, [&](const RoadSegment &s) {
        if (s.roadClass == 3 && bestD < 150) return; // prefer real roads over alleys when close
        // point-to-segment distance (local metres)
        float kx = kxAt(lat);
        float ax = (s.startLonE7 - pLon) * 1e-7f * kx, ay = (s.startLatE7 - pLat) * 1e-7f * 110540.0f;
        float bx = (s.endLonE7 - pLon) * 1e-7f * kx, by = (s.endLatE7 - pLat) * 1e-7f * 110540.0f;
        float vx = bx - ax, vy = by - ay, L2 = vx * vx + vy * vy;
        float t = L2 > 0 ? -(ax * vx + ay * vy) / L2 : 0;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        float cx = ax + t * vx, cy = ay + t * vy, d = sqrtf(cx * cx + cy * cy);
        if (d < bestD && (canFwd(s) || canBwd(s))) {
            bestD = d;
            best.s = s;
            best.fwd = canFwd(s);
            best.valid = true;
        }
    });
    if (!best.valid || bestD > 3000) {
        Serial.printf("[roadsim] no road within 3 km of %.5f,%.5f\n", (double)lat, (double)lon);
        return false;
    }
    sCur = best;
    sPosM = 0;
    sSpeedKmh = 0;
    sHeading = sCur.hdg();
    sNext = chooseNext(sCur);
    Serial.printf("[roadsim] start on seg %lu (%.0f m away, limit %d km/h, class %d), x%.2f speed\n",
                  (unsigned long)sCur.s.id, (double)bestD, sCur.s.speedLimitKmh, sCur.s.roadClass, (double)sFactor);
    return true;
}

bool roadSimStep(float *lat, float *lon, float *headingDeg, float *speedKmh) {
    if (sReqStop) {
        sReqStop = false;
        if (sActive) Serial.println("[roadsim] stopped");
        sActive = false;
    }
    if (sReqStart) {
        sReqStart = false;
        sFactor = sReqFactor;
        sActive = snapStart(sReqLat, sReqLon);
        sLastStepMs = millis();
    }
    if (!sActive) return false;

    uint32_t now = millis();
    float dt = (now - sLastStepMs) / 1000.0f;
    sLastStepMs = now;
    if (dt > 0.5f) dt = 0.5f;

    // Speed: cruise at the limit (x factor), capped so we can brake down to
    // the turn speed of the upcoming junction by the end of this segment.
    float len = sCur.len();
    float remain = len - sPosM;
    float target = cruiseKmh(sCur.s) * sFactor;
    if (sNext.valid) {
        float vT = turnKmh(angDiff(sNext.hdg(), sCur.hdg())) / 3.6f;
        float vCap = sqrtf(vT * vT + 2.0f * 2.5f * (remain > 0 ? remain : 0)) * 3.6f;
        if (vCap < target) target = vCap;
    }
    float v = sSpeedKmh;
    if (v < target) v = fminf(target, v + 6.0f * dt); // ~1.7 m/s^2
    else v = fmaxf(target, v - 11.0f * dt);          // ~3 m/s^2
    sSpeedKmh = v;

    float dM = v / 3.6f * dt;
    int guard = 0;
    while (sPosM + dM >= len && guard++ < 50) { // cross nodes (short segments)
        dM -= (len - sPosM);
        sPosM = 0;
        if (!sNext.valid) {
            Serial.println("[roadsim] stuck (no way out) — re-snapping");
            if (!snapStart(sCur.toLat() * 1e-7f, sCur.toLon() * 1e-7f)) { sActive = false; return false; }
            break;
        }
        sCur = sNext;
        sNext = chooseNext(sCur);
        len = sCur.len();
    }
    sPosM += dM;
    float t = len > 0.01f ? sPosM / len : 0;
    float la = (sCur.fromLat() + (sCur.toLat() - sCur.fromLat()) * t) * 1e-7f;
    float lo = (sCur.fromLon() + (sCur.toLon() - sCur.fromLon()) * t) * 1e-7f;

    // Heading eases toward the segment bearing (<=120 deg/s) so the heading-up
    // map turns smoothly instead of snapping at every node.
    float h = sCur.hdg(), d = fmodf(h - sHeading + 540.0f, 360.0f) - 180.0f, maxD = 120.0f * dt;
    sHeading = fmodf(sHeading + (d > maxD ? maxD : (d < -maxD ? -maxD : d)) + 360.0f, 360.0f);

    if (now - sLastLogMs > 5000) {
        sLastLogMs = now;
        const char *name = sdMgrGetSegmentRoadName(sCur.s.id);
        Serial.printf("[roadsim] %.6f,%.6f hdg %.0f %.0f km/h (limit %d) seg %lu %s\n", (double)la, (double)lo,
                      (double)sHeading, (double)v, sCur.s.speedLimitKmh, (unsigned long)sCur.s.id, name ? name : "");
    }
    *lat = la;
    *lon = lo;
    *headingDeg = sHeading;
    *speedKmh = v;
    return true;
}

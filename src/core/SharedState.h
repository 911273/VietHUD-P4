#pragma once
#include <Arduino.h>

struct GnssSnapshot {
    float egoSpeedKmh = 50;   // filtered (median + EMA — see gnss/GNSS.cpp), what consumers should use
    // Unfiltered (no median/EMA), but WITH cfg.gnssSpeedCalibrationPct's
    // correction already applied (gnss/GNSS.cpp) — published for Settings >
    // Sensors diagnostics only, so "raw" here means "pre-filter", not
    // "unadjusted module output".
    float rawSpeedKmh = 0;
    bool fix = true;          // a real, current position+speed fix — this is what safety logic (speeding
                                // overlay, GNSS status) must key off, NOT linkAlive below.
    // Purely a UI/diagnostic signal — distinguishes "module is alive and
    // sending valid NMEA, just hasn't found satellites yet" (normal,
    // expected while indoors/cold-starting) from "not receiving anything at
    // all" (an actual wiring/power/baud problem). Added 2026-09-15 because
    // both cases used to show identically as "FAULT" and that's misleading
    // — see gnss/GNSS.cpp for how it's derived.
    bool linkAlive = false;
    int satCount = 0;

    // Real GNSS time/position for the Dashboard clock (spec section 15.2).
    // UTC only; local offset is estimated from longitude, see
    // gnss/GNSS.cpp / ui/Dashboard.cpp.
    bool timeValid = false;
    int utcHour = 0, utcMinute = 0;
    float lonDeg = 0;
    float latDeg = 0;
    // GNSS altitude above mean sea level (GGA). Absolute accuracy is poor
    // (+-10 m or worse), but its CHANGE over tens of seconds reliably shows a
    // car climbing onto / descending off an elevated road (map matcher).
    float altitudeM = 0;
    bool altitudeValid = false;
    float hdop = 0;       // horizontal dilution of precision (0 = unknown); jumps under viaducts
    uint32_t fixSeq = 0;  // increments on every NEW position fix (the matcher's HMM steps once per fix)
    bool fromPhone = false; // P4: fix comes from the phone over BLE (net/PhoneGpsBle.h), badge "PHONE"
    // Sunrise/sunset estimate from GNSS date+time+position (spec section
    // 15.2: "no light sensor" — this IS that calculation). Only meaningful
    // when timeValid; defaults true (sun) so a not-yet-fixed clock icon
    // shows something neutral rather than defaulting to night.
    bool daytime = true;

    // Course over ground, degrees, 0=North/clockwise (standard NMEA COG) —
    // added 2026-09-15 for map/SpeedLimitManager.cpp's map-matching (the
    // architecture this was requested against: "M10N xác định vị trí, tốc
    // độ, hướng" — position+speed were already here, heading wasn't). Only
    // meaningful when headingValid (a GPS module reports course from
    // consecutive fixes, not a compass — unreliable/noisy near-stationary,
    // same reasoning speed itself needs the median+EMA filter in
    // gnss/GNSS.cpp).
    bool headingValid = false;
    float headingDeg = 0;
    // True when headingDeg is a brief DEAD-RECKONED HOLD of the last live
    // GPS course rather than a fresh reading this tick (gnss/GNSS.cpp,
    // kHeadingHoldMs) — added 2026-09-22 ("dự đoán hướng di chuyển của xe")
    // because a real GPS course briefly drops out (module reports invalid,
    // or the car slows below kGnssMotionThresholdKmh) exactly at
    // intersections/overpasses/underpasses, which used to cost
    // map/SpeedLimitManager.cpp its only way to disambiguate crossing roads
    // right when it needed it most. map/SpeedLimitManager.cpp's
    // evaluateSegment() still uses headingDeg for scoring when this is true
    // (a held heading still tells a perpendicular cross-street from the
    // road ahead) but caps confidence slightly below a live reading's, since
    // a hold is a prediction, not a measurement. Meaningless when
    // !headingValid.
    bool headingPredicted = false;
};

// Published by map/SpeedLimitManager.cpp — the ESP32-side "map matching"
// half of the architecture (M10N gives position+speed+heading; this
// cross-references it against a speed-limit database loaded from the
// onboard microSD, built offline on a PC by tools/map_builder/ — see
// map/SpeedMapFormat.h and map/SpeedLimitManager.h for the full picture).
// HONESTY NOTE, same as every other sensor in this project before real
// hardware existed for it: `valid` legitimately stays false until a real
// card with a real database is wired in and a GNSS fix is matched against
// it — that's correct, honest behavior (ui/Dashboard.cpp shows "--"), not a
// bug.
struct RoadInfoSnapshot {
    bool mapLoaded = false;   // an SD card was mounted and a database was validated at boot
    bool valid = false;       // a segment WITH a known speed limit was matched to the current GNSS fix just now
    float speedLimitKmh = -1; // meaningless unless valid
    float matchDistanceM = -1; // perpendicular distance from the fix to the matched segment, diagnostic only

    // Diagnostic-only fields (Settings > Sensors > Speed Map, /api/speedlimit,
    // /api/speedmap/debug) — NOT shown on the Dashboard itself (spec section
    // 24: source/confidence/road ID belong in Settings/diagnostics, not the
    // driving screen). `source` is a SpeedSource enum value (see
    // map/SpeedMapFormat.h) kept as a raw uint8_t here rather than pulling
    // that enum into this generic pub/sub header — map/SpeedLimitManager.h's
    // speedSourceStr() converts it for display.
    uint8_t source = 0;
    float confidence = 0; // 0.0-1.0, meaningful even when !valid (e.g. a road matched but with no known speed limit)
    uint32_t roadId = 0;

    // Upcoming speed-limit-change lookahead (user-requested 2026-09-21,
    // "canh bao gioi han toc do doan duong tiep theo, bao truoc khoang
    // 100m") — map/SpeedLimitManager.cpp projects ~100m ahead along the
    // current heading and re-matches THAT point; these two are only ever
    // set true/valid when that ahead-match found a road with a speed limit
    // DIFFERENT from `speedLimitKmh` above (re-confirming the same limit
    // ahead isn't a "change" worth warning about). Shown on the Dashboard
    // as a small "upcoming limit" hint next to the current sign.
    bool aheadLimitValid = false;
    float aheadSpeedLimitKmh = -1; // meaningless unless aheadLimitValid
    // REMAINING distance to where the limit changes, counting down as the
    // vehicle approaches (map/SpeedLimitManager.cpp's findAheadLimitChange()
    // scans outward in steps and reports the first differing sample, so this
    // is quantized to that step — 20m at the default range — not continuous).
    // Until 2026-09-22 this carried the configured lookahead distance itself,
    // which meant the UI permanently read "in 100m" however close the change
    // actually was.
    float aheadDistanceM = -1; // meaningless unless aheadLimitValid

    // Upcoming speed-camera warning (feature-requested 2026-09-21, "tai du
    // lieu ve canh bao giao thong, gom camera cung nhu gioi han toc do") —
    // map/SpeedLimitManager.cpp's own separate scan over the CameraPoint[]
    // loaded from /speedmap/cameras.bin (map/SdCardManager.cpp), completely
    // independent of the road-segment matching above (a camera can be
    // "ahead" regardless of whether the current road segment itself
    // matched). Sourced from OSM's highway=speed_camera tag / the WYN sign
    // CSV's sign_type=4 rows — see CameraPoint's own SpeedMapFormat.h
    // comment.
    bool cameraAheadValid = false;
    float cameraAheadDistanceM = -1;  // meaningless unless cameraAheadValid
    float cameraSpeedLimitKmh = -1;   // the camera's own tagged limit if OSM/WYN had one, else whatever road speedLimitKmh above says (may itself be -1/unknown)

    // Traffic alerts (Khu dân cư, cấm vượt, trạm thu phí, đèn tín hiệu)
    bool residentArea = false; // true if inside resident area
    bool residentAreaAheadValid = false;
    float residentAreaAheadDistM = -1;
    bool residentAreaIsStart = true; // true=R.420 (start), false=R.421 (end)

    bool noOvertaking = false; // true if inside no-overtake zone
    bool noOvertakingAheadValid = false;
    float noOvertakingAheadDistM = -1;
    bool noOvertakingIsStart = true; // true=P.125 (start), false=DP.133 (end)

    bool tollBoothAheadValid = false;
    float tollBoothAheadDistM = -1;

    bool trafficLightAheadValid = false;
    float trafficLightAheadDistM = -1;

    // Hazard / danger zone ahead (SIGN_TYPE_DANGER_OTHER = 10: đoạn đường nguy
    // hiểm / đường hầm / trạm dừng). Added 2026-09-24 — 7721 such points existed
    // in the sign database but were never matched or warned. No sub-type in the
    // data distinguishes tunnel vs. rest-stop vs. generic hazard, so it's warned
    // generically ("đoạn đường nguy hiểm phía trước").
    bool dangerAheadValid = false;
    float dangerAheadDistM = -1;

    // General upcoming sign hint
    uint8_t nextSignType = 0;
    float nextSignDistanceM = -1;
    uint8_t nextSignSpeedLimit = 0;

    // Street name of the current road (e.g. "Đ. Nguyễn Trãi", "QL 1A", "Đại lộ Thăng Long")
    char roadName[48] = {0};
};

// One pre-transformed screen-space road segment for the live background map
// (user-requested 2026-09-22, "tich hop ban do vector offline lam nen") —
// map/MapRenderer.cpp does the flat-earth + heading-up rotation + zoom-scale
// math on Core 0 and hands ui/Dashboard.cpp already-screen-space endpoints,
// so the UI task (Core 1) only ever draws lines, never touches lat/lon or
// trig. int16_t is enough for any screen coordinate this project's displays
// use (max 480) with room to spare for a point just off-screen that a line
// still needs to reach.
struct MapLine {
    int16_t x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    // RoadSegment::roadClass (0-3), passed through for ui/Dashboard.cpp's
    // per-class line style — EXCEPT the sentinel value 255, which
    // map/MapRenderer.cpp sets instead whenever this line's segment id
    // matches RoadInfoSnapshot::roadId (the road the live matcher says the
    // car is currently on) — spec's "đoạn đường xe đang chạy: highlight
    // phát sáng màu xanh cyan neon", overriding the class-based color for
    // just that one segment.
    uint8_t roadClass = 0;
};
static const uint8_t kMapLineCurrentRoad = 255;

// One upcoming sign/camera marker's screen position — `kind` mirrors
// ui/Dashboard.cpp's own AlertKind ordering closely enough to pick an icon,
// but is kept as a raw uint8_t here rather than pulling that enum into this
// generic pub/sub header, same reasoning RoadInfoSnapshot::source already
// documents for SpeedSource.
struct MapMarker {
    int16_t x = 0, y = 0;
    uint8_t kind = 0;
};

// Published by map/MapRenderer.cpp, throttled to real movement (own
// comment on mapRendererUpdate() has the exact gate) rather than every
// tick — `generation` only increments when a NEW frame's worth of geometry
// was actually computed, so ui/Dashboard.cpp's updateMapCanvas() can skip
// redrawing (and the ~33ms full-panel flush that would trigger — see
// display/DisplayDriver.h) on every one of its own 150ms ticks that finds
// nothing new here.
struct MapViewSnapshot {
    bool valid = false; // false whenever !gnss.fix or no map database is loaded — Dashboard dims/hides honestly, same convention every other reading here follows
    uint32_t generation = 0;
    float zoomRadiusM = 300;

    // Heading-up map (user-requested 2026-09-24, "hướng xe chạy luôn là góc 12
    // giờ với màn hình, bản đồ sẽ tự xoay và dịch chuyển"). map/MapRenderer.cpp
    // now projects the vector layer NORTH-UP (heading not baked into the screen
    // coords) and publishes here the heading + the exact center it projected
    // about; ui/Dashboard.cpp renders the raster background north-up about the
    // same center/scale, then rotates the WHOLE map canvas (LVGL image
    // rotation) by -headingUpDeg so travel points at 12 o'clock and raster +
    // vector rotate together, perfectly aligned. headingUpDeg is the smoothed
    // course used; egoLat/LonDeg is the snapped center (so the raster centers
    // exactly where the vector projection does).
    float headingUpDeg = 0;
    float egoLatDeg = 0;
    float egoLonDeg = 0;

    static const int kMaxLines = 180;
    int lineCount = 0;
    MapLine lines[kMaxLines];

    static const int kMaxTrailPoints = 24;
    int trailCount = 0;
    int16_t trailX[kMaxTrailPoints] = {0};
    int16_t trailY[kMaxTrailPoints] = {0};

    static const int kMaxMarkers = 12;
    int markerCount = 0;
    MapMarker markers[kMaxMarkers];
};

// Mutex-protected publish/snapshot pairs (spec docs/V1.2_hardening_proposal.md
// section A.2 "TargetBuffer" pattern). gnss/GNSS.cpp, map/SpeedLimitManager.cpp
// and map/MapRenderer.cpp publish; the UI task only ever reads a copy,
// never a pointer into live data, so a render can never race a mid-update
// struct.
// Latest ESP32-S3 die temperature (°C), published by ui/Dashboard.cpp's thermal
// block every ~2s and read by net/WebPortal.cpp for the web telemetry. A single
// float write/read is atomic on this MCU, so no mutex is needed. 0 until the
// first reading.
extern volatile float g_boardTempC;

void sharedStateInit();

void gnssPublish(const GnssSnapshot &s);
GnssSnapshot gnssSnapshot();

void roadInfoPublish(const RoadInfoSnapshot &s);
RoadInfoSnapshot roadInfoSnapshot();

void mapViewPublish(const MapViewSnapshot &s);
MapViewSnapshot mapViewSnapshot();

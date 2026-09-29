#include "SpeedLimitManager.h"
#include "SdCardManager.h"
#include "RoutePredictor.h"
#include "TrackContinuity.h" // HMM layer/continuity tracking (elevated vs surface roads) // Route — the forward-route engine (SD-free, unit-testable); see that header for why it's separate
#include "core/AppConfig.h" // cfg.aheadLimitWarnDistM / cfg.cameraWarnDistM — user-tunable, see that file's own comment
#include "core/SharedState.h"
#include "gnss/GNSS.h" // kGnssMotionThresholdKmh — ahead-lookahead only runs while actually moving
#include "MapRenderer.h" // mapRendererUpdate() — live background map, driven from this same task loop
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

// ---------------------------------------------------------------------
// Tunable matching constants — concrete defaults for the spec's qualitative
// rules (sections 13-17), documented in the plan
// (C:\Users\phamq\.claude\plans\wobbly-swinging-chipmunk.md). Not yet
// exposed in Settings/AppConfig — same "ship a reasoned default, make it
// tunable once real data exists to tune against" approach this project used
// for the LD2451's own calibration before Settings > Radar existed.
// ---------------------------------------------------------------------
static const float kMaxMatchDistanceM = 55.0f;  // candidates farther than this from the fix aren't considered at all
static const float kMinConfidence = 0.5f;       // below this, a match isn't trusted enough to switch to or report
static const float kSwitchMargin = 0.15f;       // a competing road must beat the current one by this much (after the continuity bonus) to steal the match
static const float kContinuityBonus = 0.1f;     // score bonus for staying on the road we were already matched to
static const uint32_t kMatchTimeoutMs = 8000;   // how long a low/no-confidence tick can hold the last good value before falling back to UNKNOWN

// 1024, not the earlier 512: a real Northern-Vietnam extract (tools/
// map_builder/, classified roads only, 2026-09-16) has a busiest single
// 0.01-degree tile with 752 segments — denser than the Hanoi-core test's
// 489-segment worst case that justified 512 in the first place. 512 would
// have silently TRUNCATED that tile's data (SdCardManager.cpp's readTile()
// caps at the buffer size it's given, with no error) rather than failing
// loudly, which is worse than just sizing for real data from the start.
static const int kMaxSegmentsPerTile = 2048;
// 12, not the original 5 (current + N/S/E/W only) — bumped 2026-09-22
// alongside speedLimitManagerGetNearbySegments() (the live background-map
// feature): that function queries a full square grid of tiles around the
// car (up to 3x3=9 for the map's largest ~1000m zoom, since tiles are
// ~1113m wide at TILE_SIZE_DEG=0.01 — see tools/map_builder/
// build_speedmap.py), including the diagonal tiles runMatch()'s own
// "plus"-shaped 5-tile neighbor set never needed at its much smaller 30m
// match radius. Sharing ONE cache between both consumers (rather than
// giving the map its own second cache/loader) only works without constant
// eviction thrashing if it's sized to hold BOTH working sets at once — 12
// comfortably covers the live matcher's 5 (which mostly overlap the map's
// own 9, being centered on the same car position) plus headroom for the
// non-overlapping corners.
static const int kCacheSize = 24; // 12 -> 24 (2026-09-27): dense areas at wide zoom thrashed 12 (10-20 SD tile reads / 3 s)

// At 2048 segments x 28 bytes x 12 cached tiles = ~672KB, this is
// PSRAM-backed (heap_caps_malloc, MALLOC_CAP_SPIRAM), not a plain static
// array in internal RAM — same reasoning main_ui_demo.cpp's LVGL draw
// buffers deliberately do the OPPOSITE (forced into internal RAM, PSRAM too
// slow for a per-frame render path), inverted: this cache is only touched
// once every ~500ms (see speedLimitTaskFn()'s loop), so PSRAM's higher
// latency is irrelevant, and freeing internal RAM matters more here —
// especially once WiFi (net/WebPortal.cpp) is toggled on, which alone costs
// tens of KB of internal RAM. ~672KB is still under 12% of the 5.7MB PSRAM
// left free after the tile index (up to ~1.66MB for a real regional
// extract) and everything else this project already keeps in PSRAM.
struct CachedTile {
    bool valid = false;
    uint32_t tileId = 0;
    uint32_t lastUsedMs = 0;
    int segCount = 0;
    RoadSegment *segments = NULL; // PSRAM buffer, allocated once in speedLimitManagerStart()
    // Node lookup (2026-09-26): segments[] is sorted by startLatE7 at load, and
    // endIdx[] lists segment indices sorted by endLatE7, so the segments
    // touching a node are found by binary search (tileSegmentsAtNode) instead
    // of scanning the whole tile — the matcher/route hot path in dense streets.
    uint16_t *endIdx = NULL;
};
static CachedTile cache[kCacheSize];

static void allocateTileCache() {
    for (int i = 0; i < kCacheSize; i++) {
        cache[i].segments =
            (RoadSegment *)heap_caps_malloc(sizeof(RoadSegment) * kMaxSegmentsPerTile, MALLOC_CAP_SPIRAM);
        if (!cache[i].segments) {
            // PSRAM exhausted or unavailable — extremely unlikely on this
            // board (8MB PSRAM, barely used elsewhere) but fall back to
            // internal RAM rather than leave a null pointer that would
            // crash the first time this slot is used.
            Serial.println("[map] WARN: PSRAM allocation for tile cache failed, falling back to internal RAM");
            cache[i].segments = (RoadSegment *)malloc(sizeof(RoadSegment) * kMaxSegmentsPerTile);
        }
        cache[i].endIdx = (uint16_t *)heap_caps_malloc(sizeof(uint16_t) * kMaxSegmentsPerTile, MALLOC_CAP_SPIRAM);
    }
}

static SpeedMapMetadata gMetadata;
static bool gMapLoaded = false;
static uint32_t currentRoadId = 0;
static float currentConfidence = 0;
static uint32_t lastValidMs = 0;
static RoadInfoSnapshot lastPublished;

bool speedLimitManagerGetInfo(SpeedMapMetadata &out) {
    if (!gMapLoaded) return false;
    out = gMetadata;
    return true;
}

const char *speedSourceStr(uint8_t source) {
    switch (source) {
        case SPEED_SOURCE_OSM_MAXSPEED: return "OSM_MAXSPEED";
        case SPEED_SOURCE_OSM_FORWARD: return "OSM_FORWARD";
        case SPEED_SOURCE_OSM_BACKWARD: return "OSM_BACKWARD";
        case SPEED_SOURCE_DEFAULT: return "DEFAULT";
        case SPEED_SOURCE_CONDITIONAL: return "CONDITIONAL";
        default: return "UNKNOWN";
    }
}

// ---------------------------------------------------------------------
// Geometry — local flat-earth projection around each segment's own start
// point. Good to a few cm of error over the ~30m match radius this is used
// at (equirectangular approximation, not a proper geodesic), which is far
// tighter than the GNSS fix's own accuracy — no need for anything more
// precise here.
// ---------------------------------------------------------------------
struct Vec2 {
    float x, y;
};

static Vec2 toLocalMeters(float latDeg, float lonDeg, float originLatDeg, float originLonDeg) {
    float dLat = latDeg - originLatDeg;
    float dLon = lonDeg - originLonDeg;
    float y = dLat * 110540.0f;
    float x = dLon * 111320.0f * cosf(originLatDeg * (float)M_PI / 180.0f);
    return {x, y};
}

static float pointSegmentDistanceM(float fixLat, float fixLon, float startLat, float startLon, float endLat,
                                    float endLon) {
    Vec2 p = toLocalMeters(fixLat, fixLon, startLat, startLon);
    Vec2 e = toLocalMeters(endLat, endLon, startLat, startLon);
    float segLenSq = e.x * e.x + e.y * e.y;
    float t = segLenSq > 1e-6f ? (p.x * e.x + p.y * e.y) / segLenSq : 0.0f;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    float dx = p.x - e.x * t, dy = p.y - e.y * t;
    return sqrtf(dx * dx + dy * dy);
}

static float angularDiffDeg(float a, float b) {
    float d = fmodf(fabsf(a - b), 360.0f);
    return d > 180.0f ? 360.0f - d : d;
}

// ---------------------------------------------------------------------
// Candidate scoring — see the plan's "Map matching algorithm" section for
// the reasoning behind each rule (direction disqualification vs. heading
// penalty, the heading-unavailable confidence cap, etc).
// ---------------------------------------------------------------------
struct MatchCandidate {
    uint32_t roadId;
    float distanceM;
    float headErrDeg; // heading error vs the segment's allowed direction(s) (0 if heading invalid)
    float confidence;
    int16_t speedLimitKmh;
    uint8_t source;
};

static bool evaluateSegment(const RoadSegment &seg, float fixLat, float fixLon, bool headingValid, float headingDeg,
                             bool headingPredicted, MatchCandidate &outCand) {
    float maxMatchDistanceM = kMaxMatchDistanceM;

    float startLat = seg.startLatE7 / 1e7f, startLon = seg.startLonE7 / 1e7f;
    float endLat = seg.endLatE7 / 1e7f, endLon = seg.endLonE7 / 1e7f;
    float dist = pointSegmentDistanceM(fixLat, fixLon, startLat, startLon, endLat, endLon);
    if (dist > maxMatchDistanceM) return false;

    float forwardHeading = (float)seg.headingDeg;
    float backwardHeading = fmodf(forwardHeading + 180.0f, 360.0f);

    float headingErr = 0;
    if (headingValid) {
        if (seg.direction == DIR_FORWARD) headingErr = angularDiffDeg(headingDeg, forwardHeading);
        else if (seg.direction == DIR_BACKWARD) headingErr = angularDiffDeg(headingDeg, backwardHeading);
        else { // BIDIRECTIONAL/UNKNOWN: whichever of the two directions of travel this segment allows is closer
            float e1 = angularDiffDeg(headingDeg, forwardHeading);
            float e2 = angularDiffDeg(headingDeg, backwardHeading);
            headingErr = e1 < e2 ? e1 : e2;
        }
    }

    // A genuine legal one-way (not just "this record only carries one
    // direction's speed value" — see SEGFLAG_ONEWAY's own comment) being
    // approached the wrong way is disqualified outright, never merely
    // penalized (spec 9/13.3). Only enforced when heading is trustworthy —
    // an unreliable heading must exclude a candidate from this check
    // entirely rather than risk silently waving through a wrong-way match.
    if ((seg.flags & SEGFLAG_ONEWAY) && headingValid && headingErr > 90.0f) return false;

    float confidence;
    if (headingValid) {
        float rad = headingErr * (float)M_PI / 180.0f;
        float cosH = cosf(rad);
        if (cosH < 0.0f) cosH = 0.0f;
        // Strong heading alignment keeps confidence high even on wide multi-lane roads
        confidence = (1.0f - dist / (maxMatchDistanceM * 1.25f)) * (0.30f + 0.70f * cosH);
        if (headingPredicted && confidence > 0.85f) confidence = 0.85f;
    } else {
        confidence = (1.0f - dist / maxMatchDistanceM) * 0.6f;
        if (confidence > 0.6f) confidence = 0.6f;
    }
    if (confidence < 0.0f) confidence = 0.0f;
    if (confidence > 1.0f) confidence = 1.0f;

    outCand.roadId = seg.id;
    outCand.distanceM = dist;
    outCand.headErrDeg = headingErr;
    outCand.confidence = confidence;
    outCand.speedLimitKmh = seg.speedLimitKmh;
    outCand.source = seg.speedSource;
    return true;
}

// ---------------------------------------------------------------------
// Tile cache — see spec sections 20-21: never read the whole database into
// RAM, only the current tile + its 4 neighbors, evicting least-recently-used
// when the small fixed cache is full.
// ---------------------------------------------------------------------
static uint32_t gTileLoadsWindow = 0; // SD tile reads (cache misses) since the last [map] tick log
static uint32_t packTile(int32_t latCell, int32_t lonCell) {
    return ((uint32_t)latCell << 16) | ((uint32_t)lonCell & 0xFFFF);
}

static CachedTile *findCachedTile(uint32_t tileId) {
    for (int i = 0; i < kCacheSize; i++)
        if (cache[i].valid && cache[i].tileId == tileId) return &cache[i];
    return NULL;
}

static const RoadSegment *gSortSegs = NULL; // qsort context for cmpEndIdx (single task)
static int cmpStartLat(const void *a, const void *b) {
    int32_t x = ((const RoadSegment *)a)->startLatE7, y = ((const RoadSegment *)b)->startLatE7;
    return (x > y) - (x < y);
}
static int cmpEndIdx(const void *a, const void *b) {
    int32_t x = gSortSegs[*(const uint16_t *)a].endLatE7, y = gSortSegs[*(const uint16_t *)b].endLatE7;
    return (x > y) - (x < y);
}
static void indexTile(CachedTile &t) {
    qsort(t.segments, t.segCount, sizeof(RoadSegment), cmpStartLat);
    if (!t.endIdx) return;
    for (int i = 0; i < t.segCount; i++) t.endIdx[i] = (uint16_t)i;
    gSortSegs = t.segments;
    qsort(t.endIdx, t.segCount, sizeof(uint16_t), cmpEndIdx);
}

// Calls fn(seg) for every segment of the tile with an endpoint coinciding with
// the node (segNodesCoincide), via the two sorted orders built by indexTile().
template <class Fn> static void tileSegmentsAtNode(const CachedTile &t, int32_t latE7, int32_t lonE7, Fn fn) {
    const int32_t lo = latE7 - SEG_NODE_MATCH_EPS_E7, hi = latE7 + SEG_NODE_MATCH_EPS_E7;
    int a = 0, b = t.segCount;
    while (a < b) { int m = (a + b) >> 1; if (t.segments[m].startLatE7 < lo) a = m + 1; else b = m; }
    for (int i = a; i < t.segCount && t.segments[i].startLatE7 <= hi; i++)
        if (segNodesCoincide(t.segments[i].startLatE7, t.segments[i].startLonE7, latE7, lonE7)) fn(t.segments[i]);
    if (!t.endIdx) { // no index memory: plain scan for the end-node side
        for (int i = 0; i < t.segCount; i++)
            if (segNodesCoincide(t.segments[i].endLatE7, t.segments[i].endLonE7, latE7, lonE7) &&
                !segNodesCoincide(t.segments[i].startLatE7, t.segments[i].startLonE7, latE7, lonE7))
                fn(t.segments[i]);
        return;
    }
    a = 0; b = t.segCount;
    while (a < b) { int m = (a + b) >> 1; if (t.segments[t.endIdx[m]].endLatE7 < lo) a = m + 1; else b = m; }
    for (int i = a; i < t.segCount; i++) {
        const RoadSegment &g = t.segments[t.endIdx[i]];
        if (g.endLatE7 > hi) break;
        // A segment whose BOTH ends touch the node (degenerate) was already reported above.
        if (segNodesCoincide(g.endLatE7, g.endLonE7, latE7, lonE7) &&
            !segNodesCoincide(g.startLatE7, g.startLonE7, latE7, lonE7))
            fn(g);
    }
}

static const CachedTile *getOrLoadTile(uint32_t tileId) {
    CachedTile *hit = findCachedTile(tileId);
    if (hit) {
        hit->lastUsedMs = millis();
        return hit;
    }

    // Only touch SD for a tile that actually exists in the index — most of
    // the 5 candidates around any given fix legitimately don't exist yet in
    // the database, and that's not an error worth a wasted file-open
    // attempt.
    //
    // Binary search, not a linear scan (switched 2026-09-16 alongside a
    // real Northern-Vietnam extract reaching 62081 index entries — a linear
    // scan of that many PSRAM reads, 5 times per match tick, was real
    // overhead a small Hanoi-core test's 165 entries never surfaced).
    // SpeedMapFormat.h's own TileIndexEntry comment documents the index as
    // sorted by tileId ascending specifically so this is valid — confirmed
    // against build_speedmap.py's build_database(), which writes entries via
    // `for tid, segs in sorted(tiles.items())`.
    TileIndexEntry entry;
    if (!sdMgrFindTileEntry(tileId, &entry)) return NULL;
// Guard the extremely-unlikely case where allocateTileCache() couldn't
    // get memory for this slot at all (PSRAM AND internal RAM both
    // exhausted) — treat it the same as "tile not cached", rather than
    // writing through a null pointer.
    int victim = 0;
    for (int i = 1; i < kCacheSize; i++) {
        if (!cache[i].valid) {
            victim = i;
            break;
        }
        if (cache[i].lastUsedMs < cache[victim].lastUsedMs) victim = i;
    }
    if (!cache[victim].segments) return NULL;
    gTileLoadsWindow++;
    int n = 0;
    sdMgrReadTile(entry, cache[victim].segments, kMaxSegmentsPerTile, &n);
    cache[victim].valid = true;
    cache[victim].tileId = tileId;
    cache[victim].segCount = n;
    cache[victim].lastUsedMs = millis();
    indexTile(cache[victim]);
    return &cache[victim];
}

// ---------------------------------------------------------------------
// Nearby-segments query for map/MapRenderer.cpp's live background map (see
// SpeedLimitManager.h's own declaration comment for why this lives here
// rather than MapRenderer.cpp reimplementing its own tile cache). Scans a
// full square grid of tiles (NOT the plus-shaped 5-neighbor set runMatch()
// uses — that's sized for a 30m match radius; this needs the diagonal tiles
// too at up to ~1000m) around (lat,lon), pre-filtering by real point-to-
// SEGMENT distance (pointSegmentDistanceM, the same helper evaluateSegment()
// uses for the live matcher) rather than distance from just the segment's
// start point. The start-point-only version (until 2026-09-22) silently
// dropped any segment whose start was >radiusM away even when the query
// point sat right on its middle or far end — real on long segments
// (highways especially) — which meant MapRenderer.cpp's matchedSeg lookup
// could fail to find the very road SpeedLimitManager's own live matcher was
// confidently on, breaking snap-to-road and the current-road highlight for
// no GPS/heading-related reason at all. Found reviewing the whole matching
// pipeline for the junction/overpass bug.
bool speedLimitManagerGetNearbySegments(float lat, float lon, float radiusM, RoadSegment *out, int maxOut,
                                         int *outCount) {
    *outCount = 0;
    if (!gMapLoaded) return false;

    // How many tile-widths the radius spans, rounded up, so a car sitting
    // right at a tile's edge/corner still gets every tile that could hold a
    // segment within radiusM. tileWidthM uses the same 111320 m/deg constant
    // toLocalMeters() already uses elsewhere in this file. In practice this
    // is always span=1 (a 3x3=9-tile grid) for every zoom tier this project
    // defines (max 1000m, vs. ~1113m tiles at the real TILE_SIZE_DEG=0.01 —
    // see tools/map_builder/build_speedmap.py); the span>3 clamp is purely a
    // defensive ceiling against a hypothetical future database built with a
    // much smaller tileSizeDeg, not something normal operation ever hits —
    // (2*3+1)^2=49 tiles would thrash even this enlarged cache if it ever
    // did.
    float tileWidthM = gMetadata.tileSizeDeg * 111320.0f;
    int span = (int)ceilf(radiusM / tileWidthM);
    if (span < 1) span = 1;
    if (span > 3) span = 3;

    int32_t latCell = (int32_t)((lat + 90.0f) / gMetadata.tileSizeDeg);
    int32_t lonCell = (int32_t)((lon + 180.0f) / gMetadata.tileSizeDeg);

    // BEST-N selection (2026-09-25). This used to keep the FIRST maxOut segments
    // in tile-scan order and stop. That was fine on the old sparse regional data
    // (a 3x3 tile window rarely held more than maxOut=180 segments), but on the
    // full-VN dataset a dense-city window holds far more, and "first 180 in scan
    // order" would drop major roads — and even the road the car is on — in favour
    // of whichever alleys happened to sort first. So instead we score EVERY
    // candidate in range and keep the best maxOut. This only feeds the visual map
    // (MapRenderer); the speed/warning matcher scans all segments directly, so it
    // is unaffected. Lower score = better:
    //   * the current road (seg.id == currentRoadId) is forced in (score -1e6),
    //   * then nearest-first (distance in metres),
    //   * with a small road-class penalty so, when the window is crowded and the
    //     nearest-N would otherwise be all alleys, major roads still make the cut.
    auto classPenaltyM = [](uint8_t roadClass) -> float {
        switch (roadClass) {
            case 1: return 0.0f;    // motorway/trunk/primary
            case 2: return 15.0f;   // secondary/tertiary
            case 3: return 40.0f;   // residential/service/alley
            default: return 60.0f;  // unclassified/other
        }
    };
    // Bounded top-N by score, kept in out[] with a parallel score[] scratch.
    // maxOut is kMaxLines (180) from the map caller; cap the scratch defensively.
    static const int kSelCap = 200;
    if (maxOut > kSelCap) maxOut = kSelCap;
    float score[kSelCap];
    int n = 0;
    // Max-heap on score (root = worst kept) — replacing the worst is O(log N).
    // The old linear "recompute the worst" after every replacement was O(N*180):
    // ~1 s per map update in dense areas at wide zoom (bench, 2026-09-27).
    auto siftDown = [&](int i) {
        for (;;) {
            int l = 2 * i + 1, r = l + 1, m = i;
            if (l < n && score[l] > score[m]) m = l;
            if (r < n && score[r] > score[m]) m = r;
            if (m == i) return;
            float ts = score[i]; score[i] = score[m]; score[m] = ts;
            RoadSegment tg = out[i]; out[i] = out[m]; out[m] = tg;
            i = m;
        }
    };
    auto siftUp = [&](int i) {
        while (i > 0) {
            int pa = (i - 1) / 2;
            if (score[pa] >= score[i]) return;
            float ts = score[i]; score[i] = score[pa]; score[pa] = ts;
            RoadSegment tg = out[i]; out[i] = out[pa]; out[pa] = tg;
            i = pa;
        }
    };
    const float cosLat = cosf(lat * (float)M_PI / 180.0f);
    const int32_t rLatE7 = (int32_t)(radiusM / 110540.0f * 1e7f);
    const int32_t rLonE7 = (int32_t)(radiusM / (111320.0f * cosLat) * 1e7f);
    const int32_t latE7 = (int32_t)(lat * 1e7f), lonE7 = (int32_t)(lon * 1e7f);
    for (int dLat = -span; dLat <= span; dLat++) {
        for (int dLon = -span; dLon <= span; dLon++) {
            const CachedTile *tile = getOrLoadTile(packTile(latCell + dLat, lonCell + dLon));
            if (!tile) continue; // most candidate tiles legitimately don't exist in the database — not an error
            for (int s = 0; s < tile->segCount; s++) {
                const RoadSegment &seg = tile->segments[s];
                // Cheap reject: both ends beyond the radius on the same side.
                if ((seg.startLatE7 > latE7 + rLatE7 && seg.endLatE7 > latE7 + rLatE7) ||
                    (seg.startLatE7 < latE7 - rLatE7 && seg.endLatE7 < latE7 - rLatE7) ||
                    (seg.startLonE7 > lonE7 + rLonE7 && seg.endLonE7 > lonE7 + rLonE7) ||
                    (seg.startLonE7 < lonE7 - rLonE7 && seg.endLonE7 < lonE7 - rLonE7))
                    continue;
                float startLat = seg.startLatE7 / 1e7f, startLon = seg.startLonE7 / 1e7f;
                float endLat = seg.endLatE7 / 1e7f, endLon = seg.endLonE7 / 1e7f;
                float d = pointSegmentDistanceM(lat, lon, startLat, startLon, endLat, endLon);
                if (d > radiusM) continue;
                float sc = (currentRoadId != 0 && seg.id == currentRoadId) ? -1e6f
                                                                           : d + classPenaltyM(seg.roadClass);
                if (n < maxOut) {
                    out[n] = seg;
                    score[n] = sc;
                    n++;
                    siftUp(n - 1);
                } else if (sc < score[0]) {
                    out[0] = seg;
                    score[0] = sc;
                    siftDown(0);
                }
            }
        }
    }
    *outCount = n;
    return true;
}

// =====================================================================
// FORWARD-ROUTE ENGINE — adapter over map/RoutePredictor.h (added 2026-09-23)
// ---------------------------------------------------------------------
// The route-building / projection / arc-length-walking LOGIC now lives in the
// standalone, SD-free, unit-testable Route class (map/RoutePredictor.h) — see
// that header for the full rationale (it replaces the old point-based +
// heading-cone matching that mis-picked the next sign on curves/junctions and
// blanked the limit through a GPS gap). This adapter is the ONLY place that
// bridges that pure Route to this module's tile cache and holds the car's live
// position on the route, so SpeedLimitManager stays the sole SD owner
// (RoutePredictor never touches SD; it sees road data only through the
// SegmentProviderFn below). Call sites keep their original names
// (routeProject / routeLimitAtDist / buildForwardRoute / ...) so the rest of
// this file is unchanged by the extraction.
// =====================================================================
static const float kRouteBuildAheadM = 700.0f; // stop extending the route once it's this far ahead (covers the 600m max dynamic warn distance + margin)
static const float kRouteLateralTolM = 35.0f;  // a camera/sign within this perpendicular distance of the route counts as "on the route"
static const float kItemLateralTolM = 25.0f;   // stricter: a camera/sign must be this close to the route's centerline AND...
static const float kItemOwnMarginM = 3.0f;     // ...no other road may be closer to it by more than this (see routeOwnsItem)
static const float kItemNearRadiusM = 30.0f;   // roads considered as competing owners of an item

static uint32_t gTickMaxUs = 0, gTickMaxMatchUs = 0, gTickMaxMapUs = 0; // worst tick per debug window
static Route gRoute;                  // the one forward-route instance this module owns
static int gRouteCount = 0;           // mirror of gRoute.count(), kept in sync by buildForwardRoute()/resetRouteState() so call sites read a plain int
static uint32_t gRouteHeadRoadId = 0; // road id the route was built from (its first segment)
static float gCarDistM = 0;           // car's arc-length position along gRoute (from the route origin)
static bool gCarOnRoute = false;      // false if the latest fix didn't project onto the route within tolerance

// SegmentProvider backed by THIS module's tile cache — RoutePredictor's only
// view of road data. Scans the 3x3 tiles around the node and returns every
// segment incident to it in EITHER orientation, raw/unoriented (Route::build
// does the orienting and the one-way/direction checks). Each segment is stored
// in the tile of its START node (confirmed in build_speedmap.py's
// build_database()), so a segment ENDING at this node starts <=~31m away —
// always inside the 3x3 neighborhood, never missed. Dedupes by id as cheap
// insurance against any future overlapping-tile duplication.
static int routeSegmentProvider(int32_t nodeLatE7, int32_t nodeLonE7, RoadSegment *out, int maxOut, void *ctx) {
    (void)ctx;
    int32_t latCell = (int32_t)((nodeLatE7 / 1e7f + 90.0f) / gMetadata.tileSizeDeg);
    int32_t lonCell = (int32_t)((nodeLonE7 / 1e7f + 180.0f) / gMetadata.tileSizeDeg);
    int n = 0;
    for (int dLat = -1; dLat <= 1; dLat++) {
        for (int dLon = -1; dLon <= 1; dLon++) {
            const CachedTile *tile = getOrLoadTile(packTile(latCell + dLat, lonCell + dLon));
            if (!tile) continue;
            // Tolerant node match (see segNodesCoincide in SpeedMapFormat.h):
            // returns candidates incident to the node even when tile-clipped
            // vector data leaves border endpoints off by a sub-meter amount,
            // so RoutePredictor can stitch across tile boundaries. Binary
            // search on the tile's sorted orders (tileSegmentsAtNode).
            tileSegmentsAtNode(*tile, nodeLatE7, nodeLonE7, [&](const RoadSegment &c) {
                if (n >= maxOut) return;
                for (int k = 0; k < n; k++)
                    if (out[k].id == c.id) return;
                out[n++] = c;
            });
        }
    }
    return n;
}

static float routeTotalLenM() { return gRoute.totalLenM(); }

static bool routeProject(float lat, float lon, float lateralTolM, float *outDistM, float *outLat, float *outLon,
                         float *outHeadingDeg = NULL) {
    return gRoute.project(lat, lon, lateralTolM, outDistM, outLat, outLon, outHeadingDeg);
}

// ---- Item ownership (2026-09-26, "nang cap chat luong do chinh xac canh bao
// giao thong theo du doan lo trinh phia truoc"). A camera/sign counts as on
// our route only if the route passes close to it AND it isn't clearly closer to
// some OTHER road — the parallel service road, the opposite carriageway of a
// divided road, the road under a flyover, a side street at a junction. The
// competing roads around each item come from the tile cache and are cached per
// item (items are static; the same few are re-checked every 500ms tick).
static const int kItemNearMax = 6;
struct ItemNearEntry {
    uint32_t key;   // item identity (kind bit | index) mixed with its coordinates
    uint8_t n;
    uint32_t ids[kItemNearMax];
    float dists[kItemNearMax];
};
static const int kItemNearCacheSize = 64;
static ItemNearEntry gItemNear[kItemNearCacheSize];
static bool gItemNearUsed[kItemNearCacheSize];

static const ItemNearEntry &itemNearRoads(uint32_t key, float lat, float lon) {
    int slot = (int)((key * 2654435761u) >> 26) & (kItemNearCacheSize - 1);
    ItemNearEntry &e = gItemNear[slot];
    if (gItemNearUsed[slot] && e.key == key) return e;
    e.key = key;
    e.n = 0;
    gItemNearUsed[slot] = true;
    float ts = gMetadata.tileSizeDeg;
    if (ts <= 0) return e;
    int32_t latCell = (int32_t)((lat + 90.0f) / ts);
    int32_t lonCell = (int32_t)((lon + 180.0f) / ts);
    // Cheap bbox reject before the exact distance: a segment's endpoints can't
    // both be farther than (radius + half its length) away; segments are short
    // (<~300m) so a generous box is enough.
    const float boxDeg = (kItemNearRadiusM + 300.0f) / 110540.0f;
    for (int dLat = -1; dLat <= 1; dLat++) {
        for (int dLon = -1; dLon <= 1; dLon++) {
            const CachedTile *tile = getOrLoadTile(packTile(latCell + dLat, lonCell + dLon));
            if (!tile) continue;
            // segments[] is sorted by startLatE7 (indexTile): scan only the latitude band.
            int32_t bandLo = (int32_t)((lat - boxDeg) * 1e7f), bandHi = (int32_t)((lat + boxDeg) * 1e7f);
            int s0 = 0, s1 = tile->segCount;
            while (s0 < s1) { int m = (s0 + s1) >> 1; if (tile->segments[m].startLatE7 < bandLo) s0 = m + 1; else s1 = m; }
            for (int s = s0; s < tile->segCount && tile->segments[s].startLatE7 <= bandHi; s++) {
                const RoadSegment &c = tile->segments[s];
                float aLon = c.startLonE7 / 1e7f;
                if (fabsf(aLon - lon) > boxDeg * 1.1f) continue;
                float d = segPointDistM(c, lat, lon);
                if (d > kItemNearRadiusM) continue;
                if (e.n < kItemNearMax) {
                    e.ids[e.n] = c.id; e.dists[e.n] = d; e.n++;
                } else { // keep the closest kItemNearMax
                    int worst = 0;
                    for (int k = 1; k < kItemNearMax; k++) if (e.dists[k] > e.dists[worst]) worst = k;
                    if (d < e.dists[worst]) { e.ids[worst] = c.id; e.dists[worst] = d; }
                }
            }
        }
    }
    return e;
}

// Arc distance (from the route origin) + road heading at an item that belongs
// to our predicted route; false if it's off-route or owned by another road.
static bool routeOwnsItem(uint32_t kindIndex, int32_t latE7, int32_t lonE7, float *outArcM, float *outHeadingDeg) {
    float lat = latE7 / 1e7f, lon = lonE7 / 1e7f;
    // Cheap first: most items within the horizon aren't near the route at all.
    if (!gRoute.project(lat, lon, kItemLateralTolM, NULL, NULL, NULL)) return false;
    uint32_t key = kindIndex ^ ((uint32_t)latE7 * 31u) ^ ((uint32_t)lonE7 * 17u);
    const ItemNearEntry &e = itemNearRoads(key, lat, lon);
    return gRoute.ownsPoint(lat, lon, kItemLateralTolM, e.ids, e.dists, e.n, kItemOwnMarginM, outArcM,
                            outHeadingDeg);
}

// True if a route position lies past a FORK the car hasn't reached yet — the
// predicted branch there is a guess (exit ramp vs mainline, Y-split), so a
// sign/limit beyond it could belong to the branch we won't take. Once the car
// passes the fork the route is rebuilt from the branch actually taken.
static bool routeBeyondUnpassedFork(float arcM) {
    float fork = gRoute.forkAtM();
    return fork < 1e8f && gCarDistM < fork && arcM > fork;
}

static bool routeFindAheadLimitChange(float fromDistM, float currentLimitKmh, float maxM, float &outDistM,
                                      float &outLimitKmh) {
    return gRoute.limitChangeAhead(fromDistM, currentLimitKmh, maxM, outDistM, outLimitKmh, /*taggedOnly=*/true);
}

static bool routeLimitAtDist(float distM, float &outLimitKmh) { return gRoute.limitAt(distM, outLimitKmh); }

// Build (or rebuild) the forward route from the matched segment. Cheap enough
// to run on demand; the caller decides when (head-road change, or running low
// on built-ahead route) — see runMatch().
static void buildForwardRoute(const RoadSegment &startSeg, float startHeading) {
    gRouteHeadRoadId = startSeg.id;
    gRouteCount = gRoute.build(startSeg, startHeading, kRouteBuildAheadM, routeSegmentProvider, NULL);
}

// Update the car's arc-length position on the current route from a fresh fix.
// Returns true if the fix projected onto the route (so gCarDistM/gCarOnRoute
// are meaningful this tick); on success routeLimitAtDist(gCarDistM) is a
// reliable current-limit source even if runMatch()'s per-segment scoring
// wobbled or matched an untagged segment.
static bool routeUpdateCarPosition(float lat, float lon) {
    float dist, snapLat, snapLon;
    if (gRoute.project(lat, lon, kRouteLateralTolM * 1.5f, &dist, &snapLat, &snapLon)) {
        gCarDistM = dist;
        gCarOnRoute = true;
        return true;
    }
    gCarOnRoute = false;
    return false;
}

// Clears all route state — used after the synthetic self-test so it doesn't
// seed the first real match with a route anchored where the car isn't.
// Passed-sign override (2026-09-26, "mot so bieu tuong canh bao toc do bi
// sai, hien so khong dung nhu bien bao"). ~96% of segments carry a DEFAULT
// limit — a guess from the road class (40/30/50), not a sign. A real speed-limit
// sign the car has just passed on its own route is ground truth, so it becomes
// the displayed limit and stays so while the car keeps following the predicted
// road ahead. It is dropped when the car turns off onto a road the route did
// not predict (a junction turn), when it loses the route, after
// kSignOverrideMaxMs, or when the next speed sign replaces it.
static float gSignOverrideKmh = -1.0f;
static uint32_t gSignOverrideMs = 0;
static uint8_t gSignOverrideSrc = SPEED_SOURCE_OSM_MAXSPEED; // DEFAULT when the value is the legal default, not a number on a sign
// Residential-area state from R.420 (start) / R.421 (end) signs passed on the
// route: -1 unknown, 0 outside, 1 inside. Sticky until the opposite sign.
static int gResidentialState = -1;
static const uint32_t kSignOverrideMaxMs = 20UL * 60UL * 1000UL;
static void clearSignOverride(const char *why) {
    if (gSignOverrideKmh > 0) Serial.printf("[map] sign limit %.0f dropped (%s)\n", (double)gSignOverrideKmh, why);
    gSignOverrideKmh = -1.0f;
}

static void resetRouteState() {
    clearSignOverride("route reset");
    gRoute.reset();
    gRouteCount = 0;
    gRouteHeadRoadId = 0;
    gCarDistM = 0;
    gCarOnRoute = false;
}


// ---------------------------------------------------------------------
// Elevated/surface road disambiguation (2026-09-26) — see map/TrackContinuity.h.
// Each NEW GNSS fix, every segment within kMaxMatchDistanceM becomes an HMM
// candidate; the tracker keeps several topologically-consistent hypotheses and
// reports the cheapest, so a car on a flyover stays on the flyover (and one
// under it stays below) instead of flipping on every GPS wobble.
// ---------------------------------------------------------------------
static const int kMaxCands = 64;
static TrackContinuity gTrack;
static RoadSegment gCandSeg[2][kMaxCands]; // this fix's and the previous fix's candidate geometry
static int gCandN[2] = {0, 0};
static int gCandCur = 0;
static uint32_t gLastFixSeq = 0;
static float gLastFixLat = 0, gLastFixLon = 0;
static uint32_t gTrackReportedId = 0;
static const char *gTrackReason = "";

static const RoadSegment *findCandSeg(uint32_t id) {
    for (int b = 0; b < 2; b++) {
        int buf = (gCandCur + b) % 2;
        for (int i = 0; i < gCandN[buf]; i++)
            if (gCandSeg[buf][i].id == id) return &gCandSeg[buf][i];
    }
    return NULL;
}

// Segments connected to `segId`: sharing a node, up to two hops, plus the next
// stretch of the forward route when segId is on it (fast cars cover >2 short
// segments between fixes). Cached per id inside TrackContinuity.
static int trackNeighbors(uint32_t segId, uint32_t *out, int maxOut, void *) {
    const RoadSegment *s = findCandSeg(segId);
    if (!s) return 0;
    int n = 0;
    auto add = [&](uint32_t id) {
        if (id == segId || n >= maxOut) return;
        for (int k = 0; k < n; k++)
            if (out[k] == id) return;
        out[n++] = id;
    };
    static RoadSegment nb1[Route::kMaxNodeCandidates], nb2[Route::kMaxNodeCandidates];
    const int32_t nodes[2][2] = {{s->startLatE7, s->startLonE7}, {s->endLatE7, s->endLonE7}};
    for (int e = 0; e < 2; e++) {
        int k1 = routeSegmentProvider(nodes[e][0], nodes[e][1], nb1, Route::kMaxNodeCandidates, NULL);
        for (int j = 0; j < k1; j++) {
            add(nb1[j].id);
            bool startsHere = segNodesCoincide(nb1[j].startLatE7, nb1[j].startLonE7, nodes[e][0], nodes[e][1]);
            int32_t farLat = startsHere ? nb1[j].endLatE7 : nb1[j].startLatE7;
            int32_t farLon = startsHere ? nb1[j].endLonE7 : nb1[j].startLonE7;
            int k2 = routeSegmentProvider(farLat, farLon, nb2, Route::kMaxNodeCandidates, NULL);
            for (int q = 0; q < k2; q++) add(nb2[q].id);
        }
    }
    for (int i = 0; i < gRouteCount; i++) {
        if (gRoute.seg(i).id != segId) continue;
        for (int j = i + 1; j < gRouteCount && j <= i + 10; j++) add(gRoute.seg(j).id);
        break;
    }
    return n;
}

// Layer evidence: GNSS altitude change over ~40 s of driving, and a sudden
// sky-view loss (HDOP well above the recent open-sky baseline).
static TrackEvidence updateTrackEvidence(const GnssSnapshot &gnss) {
    static float altHist[64];
    static uint32_t altMs[64];
    static int altN = 0, altHead = 0;
    static float hdopBase = 0;
    TrackEvidence ev;
    uint32_t now = millis();
    if (gnss.altitudeValid) {
        altHist[altHead] = gnss.altitudeM;
        altMs[altHead] = now;
        altHead = (altHead + 1) % 64;
        if (altN < 64) altN++;
        // oldest sample that is 35..60 s old
        for (int k = 0; k < altN; k++) {
            int i = (altHead - altN + k + 64) % 64;
            uint32_t age = now - altMs[i];
            if (age <= 60000 && age >= 35000) {
                ev.altValid = gnss.egoSpeedKmh > 15.0f; // only meaningful while actually driving
                ev.climbM = gnss.altitudeM - altHist[i];
                break;
            }
        }
    }
    if (gnss.hdop > 0) {
        if (gnss.hdop < 2.0f) hdopBase = hdopBase == 0 ? gnss.hdop : 0.98f * hdopBase + 0.02f * gnss.hdop;
        ev.skyBlocked = hdopBase > 0 && gnss.hdop > 2.0f && gnss.hdop > 1.8f * hdopBase;
    }
    return ev;
}

// ---------------------------------------------------------------------
// Top-level match — reads lastPublished (previous tick's result) for the
// hold-on-low-confidence behavior but never writes it; the caller (the task
// loop, or runSelfTest()) owns updating lastPublished after each call. See
// SpeedLimitManager.h's file comment and the plan for the overall flow.
// ---------------------------------------------------------------------
// Hold the last known-good match for up to kMatchTimeoutMs — shared by
// "still have a fix but nothing scored well enough this tick" and "fix
// dropped out entirely" (see runMatch()'s two call sites below). Originally
// only the first case held anything (spec section 17/32); fix loss used to
// return UNKNOWN immediately (spec section 30's original reasoning: don't
// show a possibly-wrong value without a real position). Extended
// 2026-09-22 to also cover a brief total fix loss — user-reported: real
// underpasses (hầm chui) often lose GPS fix completely for a few seconds,
// which used to blank the speed-limit display exactly there even though the
// road (and its limit) obviously hasn't changed underground. Same
// kMatchTimeoutMs cap as before bounds how long a stale value can survive —
// this is a display convenience for a SHORT gap, not a claim of continued
// certainty.
static void holdLastKnownMatch(RoadInfoSnapshot &out) {
    if (currentRoadId != 0 && millis() - lastValidMs < kMatchTimeoutMs) {
        out.valid = lastPublished.valid;
        out.speedLimitKmh = lastPublished.speedLimitKmh;
        out.source = lastPublished.source;
        out.confidence = lastPublished.confidence;
        out.roadId = currentRoadId;
        out.matchDistanceM = -1;
        strncpy(out.roadName, lastPublished.roadName, sizeof(out.roadName) - 1);
        out.roadName[sizeof(out.roadName) - 1] = '\0';
    }
}

// --- GPS-gap dead-reckoning along the route (issue #3, 2026-09-23) ---
// While the fix is lost, the old code held the last value for kMatchTimeoutMs
// then blanked. But the car is still driving the SAME route — so instead of
// freezing, advance the car's arc-length position along the persisted forward
// route using its last known speed. The limit (and ahead warnings) then follow
// the ROUTE, not the last point: through a tunnel the displayed limit stays
// correct and even updates if the route's limit changes ahead, and the moment
// GPS returns the fresh fix simply re-snaps onto (or rebuilds) that route.
// Bounded by kDeadReckonMaxMs so a long/ambiguous outage can't carry the car
// arbitrarily far on a guess — past that we honestly fall back to hold/blank.
static const uint32_t kDeadReckonMaxMs = 20000; // dead-reckon at most ~20s into a GPS gap
static float gLastKnownSpeedKmh = 0;            // last filtered speed while we had a fix
static uint32_t gFixLostMs = 0;                 // millis() when the current fix gap began (0 = have fix)
static uint32_t gLastDeadReckonMs = 0;          // last dead-reckon step's timestamp (0 = none yet this gap)

// Advance gCarDistM along the route by the distance covered since the last
// tick, using the last known speed. Returns true if the car is still on the
// built route (so routeLimitAtDist(gCarDistM) is meaningful).
static bool deadReckonAlongRoute(uint32_t nowMs) {
    if (gRouteCount == 0 || !gCarOnRoute) return false;
    if (nowMs - gFixLostMs > kDeadReckonMaxMs) return false;
    float dtSec = gLastDeadReckonMs ? (float)(nowMs - gLastDeadReckonMs) / 1000.0f : 0.0f;
    gLastDeadReckonMs = nowMs;
    if (dtSec <= 0 || dtSec > 2.0f) return gCarOnRoute; // skip a suspiciously large first/stalled step
    gCarDistM += (gLastKnownSpeedKmh / 3.6f) * dtSec;
    if (gCarDistM > gRoute.totalLenM()) {
        // Ran off the far end of the built route — stop advancing; the limit at
        // the end is the best honest guess (routeLimitAtDist clamps to last).
        gCarDistM = gRoute.totalLenM();
    }
    return true;
}

static void runMatch(const GnssSnapshot &gnss, RoadInfoSnapshot &out) {
    out.mapLoaded = true;

    if (!gnss.fix) {
        uint32_t nowMs = millis();
        if (gFixLostMs == 0) gFixLostMs = nowMs; // mark the start of this gap
        // After a long gap (tunnel, parking garage) the old hypotheses are stale:
        // start fresh so re-acquisition isn't biased by where we were.
        if (nowMs - gFixLostMs > 20000 && gTrack.hypothesisCount() > 0) gTrack.reset();
        // Try to keep the limit alive from the route (dead-reckoned forward);
        // fall back to the short hold only if there's no usable route.
        bool drOk = deadReckonAlongRoute(nowMs);
        if (drOk) {
            float rl;
            if (routeLimitAtDist(gCarDistM, rl)) {
                out.valid = true;
                out.speedLimitKmh = rl;
                out.source = SPEED_SOURCE_OSM_MAXSPEED;
                out.roadId = currentRoadId;
                out.confidence = lastPublished.confidence;
                out.matchDistanceM = -1;
                strncpy(out.roadName, lastPublished.roadName, sizeof(out.roadName) - 1);
                out.roadName[sizeof(out.roadName) - 1] = '\0';
                return;
            }
        }
        holdLastKnownMatch(out);
        return;
    }
    // We have a fix: end any gap, record the live speed for future
    // dead-reckoning, and clear the dead-reckon step clock so the first step of
    // the NEXT gap measures from that gap's start, not a stale timestamp.
    gFixLostMs = 0;
    gLastDeadReckonMs = 0;
    gLastKnownSpeedKmh = gnss.egoSpeedKmh;

    int32_t latCell = (int32_t)((gnss.latDeg + 90.0f) / gMetadata.tileSizeDeg);
    int32_t lonCell = (int32_t)((gnss.lonDeg + 180.0f) / gMetadata.tileSizeDeg);
    uint32_t tileIds[5] = {packTile(latCell, lonCell), packTile(latCell + 1, lonCell),
                            packTile(latCell - 1, lonCell), packTile(latCell, lonCell + 1),
                            packTile(latCell, lonCell - 1)};

    MatchCandidate best = {};
    bool haveBest = false;

    // Collect every candidate of this fix (geometry kept for the forward route
    // and the tracker's neighbour lookups). Over kMaxCands (very dense junction
    // areas) the farthest ones are dropped.
    static MatchCandidate cands[kMaxCands];
    int nextBuf = 1 - gCandCur;
    RoadSegment *segBuf = gCandSeg[nextBuf];
    int nc = 0;
    for (int i = 0; i < 5; i++) {
        const CachedTile *tile = getOrLoadTile(tileIds[i]);
        if (!tile) continue;
        for (int s = 0; s < tile->segCount; s++) {
            MatchCandidate cand;
            if (!evaluateSegment(tile->segments[s], gnss.latDeg, gnss.lonDeg, gnss.headingValid, gnss.headingDeg,
                                  gnss.headingPredicted, cand))
                continue;
            if (!haveBest || cand.confidence > best.confidence) {
                best = cand;
                haveBest = true;
            }
            int slot = nc;
            if (nc >= kMaxCands) {
                int far = 0;
                for (int k = 1; k < nc; k++)
                    if (cands[k].distanceM > cands[far].distanceM) far = k;
                if (cand.distanceM >= cands[far].distanceM) continue;
                slot = far;
            } else {
                nc++;
            }
            cands[slot] = cand;
            segBuf[slot] = tile->segments[s];
        }
    }
    gCandN[nextBuf] = nc;
    gCandCur = nextBuf;

    // HMM step — only on a NEW fix (this task runs faster than the GNSS rate;
    // feeding the same fix twice would double-count its evidence).
    bool newFix = gnss.fixSeq != gLastFixSeq || gnss.latDeg != gLastFixLat || gnss.lonDeg != gLastFixLon;
    int pick = -1;
    if (nc > 0 && newFix) {
        gLastFixSeq = gnss.fixSeq;
        gLastFixLat = gnss.latDeg;
        gLastFixLon = gnss.lonDeg;
        static TrackCand tc[kMaxCands];
        for (int i = 0; i < nc; i++)
            tc[i] = {cands[i].roadId, cands[i].distanceM, cands[i].headErrDeg, gnss.headingValid, segBuf[i].flags};
        TrackEvidence ev = updateTrackEvidence(gnss);
        pick = gTrack.step(tc, nc, ev, trackNeighbors, NULL, &gTrackReason);
        if (pick >= 0) gTrackReportedId = cands[pick].roadId;
    } else if (nc > 0) {
        for (int i = 0; i < nc; i++)
            if (cands[i].roadId == gTrackReportedId) { pick = i; break; }
        if (pick < 0) { // reported segment fell out of range between fixes: take the tracker's view next fix
            int bi = 0;
            for (int i = 1; i < nc; i++)
                if (cands[i].confidence > cands[bi].confidence) bi = i;
            pick = bi;
        }
    }

    MatchCandidate chosen;
    RoadSegment chosenSeg = {};
    bool haveChosen = false;
    if (pick >= 0) {
        chosen = cands[pick];
        chosenSeg = segBuf[pick];
        haveChosen = true;
    }

    if (!haveChosen || (chosen.confidence < kMinConfidence && (!haveBest || best.confidence < 0.38f))) {
        // Even though the per-fix segment match failed this tick, try to keep
        // the limit alive from the forward route built on a recent good match —
        // the car is still (almost certainly) on that route, this is just a
        // transient positioning wobble (or the first fix back after a GPS gap,
        // which often lands slightly off before it settles). Project THIS fresh
        // fix onto the route and read the limit at the car's new arc position.
        // This is the route-based half of the GPS-gap fix: the limit comes back
        // the instant a position does, without waiting to drive over a fresh
        // segment or sign.
        holdLastKnownMatch(out);
        float d, sl, sn;
        if (gRouteCount > 0 && routeProject(gnss.latDeg, gnss.lonDeg, kRouteLateralTolM * 2.0f, &d, &sl, &sn)) {
            gCarDistM = d;
            gCarOnRoute = true;
            float rl;
            if (routeLimitAtDist(d, rl)) {
                out.valid = true;
                out.speedLimitKmh = rl;
                out.source = SPEED_SOURCE_OSM_MAXSPEED;
            }
        }
        return;
    }

    currentRoadId = chosen.roadId;
    currentConfidence = chosen.confidence;
    lastValidMs = millis();

    // --- Forward route: build it from the chosen segment, then snap the car.
    // Rebuild when the head segment changes (we turned onto a new road) or the
    // car has drifted kRouteRebuildDistM along/off the existing route;
    // otherwise just refresh the car's arc-length position (cheap projection).
    bool needRebuild = (gRouteCount == 0) || (gRouteHeadRoadId != chosen.roadId);
    if (!needRebuild) {
        // If the car projects onto the existing route and hasn't run off its
        // far end, reuse it. Otherwise rebuild.
        float d, sl, sn;
        if (routeProject(gnss.latDeg, gnss.lonDeg, kRouteLateralTolM * 1.5f, &d, &sl, &sn)) {
            // Moved far enough along that we'd want fresh segments ahead?
            needRebuild = (routeTotalLenM() - d) < (kRouteBuildAheadM * 0.4f);
        } else {
            needRebuild = true; // off the route entirely
        }
    }
    if (gSignOverrideKmh > 0 && gRouteCount > 0 && gRouteHeadRoadId != chosen.roadId &&
        !gRoute.containsSegment(chosen.roadId))
        clearSignOverride("turned off the predicted road");
    if (needRebuild) {
        float hd = gnss.headingValid ? gnss.headingDeg : (float)chosenSeg.headingDeg;
        buildForwardRoute(chosenSeg, hd);
    }
    routeUpdateCarPosition(gnss.latDeg, gnss.lonDeg);

    out.roadId = chosen.roadId;
    out.confidence = chosen.confidence;
    out.matchDistanceM = chosen.distanceM;
    out.source = chosen.source;
    const char *stName = sdMgrGetSegmentRoadName(chosen.roadId);
    if (stName && stName[0]) {
        strncpy(out.roadName, stName, sizeof(out.roadName) - 1);
        out.roadName[sizeof(out.roadName) - 1] = '\0';
    } else {
        out.roadName[0] = '\0';
    }
    if (chosen.speedLimitKmh >= 0) {
        out.valid = true;
        out.speedLimitKmh = (float)chosen.speedLimitKmh;
    } else {
        // Matched a real road, just one with no known speed limit tag on this
        // exact segment — but the route around it is fully tagged (every
        // segment in this dataset carries a limit), so recover the limit from
        // the car's arc-length position on the route rather than showing a
        // blank. Falls back to an honest UNKNOWN only if the route can't say
        // either.
        float rl;
        if (gCarOnRoute && routeLimitAtDist(gCarDistM, rl)) {
            out.valid = true;
            out.speedLimitKmh = rl;
            out.source = SPEED_SOURCE_OSM_MAXSPEED;
        } else {
            out.valid = false;
            out.speedLimitKmh = -1;
        }
    }
}

// ---------------------------------------------------------------------
// Upcoming speed-limit-change lookahead (user-requested 2026-09-21, "bo
// sung chuc nang canh bao gioi han toc do doan duong tiep theo, bao truoc
// khoang 100m"). Projects a point cfg.aheadLimitWarnDistM ahead along the
// current heading (same flat-earth approximation as toLocalMeters above,
// inverted — fine at this short a range) and runs the SAME segment-matching
// scoring the live position match uses (evaluateSegment/getOrLoadTile), just
// anchored at that projected point instead of the real fix. Deliberately
// no hysteresis/continuity/hold-timeout the way runMatch()'s live tracking
// has: this is a one-shot snapshot query, recomputed fresh every tick, not
// a continuously-tracked state — a transient bad read here just means one
// tick without an ahead-warning, not a wrong CURRENT-road decision.
// User-tunable (2026-09-22, "hieu chinh khoang cach canh bao toc do phia
// truoc") — was a fixed 100m constant here, now AppConfig.h's
// cfg.aheadLimitWarnDistM (Settings > Sensors), clamped 50-300m, which is
// the FARTHEST this scans rather than the single distance it samples; see
// findAheadLimitChange() below.
static bool matchAheadPoint(const GnssSnapshot &gnss, float lookaheadM, MatchCandidate &outBest) {
    float rad = gnss.headingDeg * (float)M_PI / 180.0f;
    float dx = lookaheadM * sinf(rad);  // east component
    float dy = lookaheadM * cosf(rad);  // north component
    float aheadLat = gnss.latDeg + dy / 110540.0f;
    float aheadLon = gnss.lonDeg + dx / (111320.0f * cosf(gnss.latDeg * (float)M_PI / 180.0f));

    int32_t latCell = (int32_t)((aheadLat + 90.0f) / gMetadata.tileSizeDeg);
    int32_t lonCell = (int32_t)((aheadLon + 180.0f) / gMetadata.tileSizeDeg);
    uint32_t tileIds[5] = {packTile(latCell, lonCell), packTile(latCell + 1, lonCell),
                            packTile(latCell - 1, lonCell), packTile(latCell, lonCell + 1),
                            packTile(latCell, lonCell - 1)};

    bool haveBest = false;
    for (int i = 0; i < 5; i++) {
        const CachedTile *tile = getOrLoadTile(tileIds[i]);
        if (!tile) continue;
        for (int s = 0; s < tile->segCount; s++) {
            MatchCandidate cand;
            if (!evaluateSegment(tile->segments[s], aheadLat, aheadLon, gnss.headingValid, gnss.headingDeg,
                                  gnss.headingPredicted, cand))
                continue;
            if (!haveBest || cand.confidence > outBest.confidence) {
                outBest = cand;
                haveBest = true;
            }
        }
    }
    return haveBest && outBest.confidence >= kMinConfidence && outBest.speedLimitKmh >= 0;
}

// Straight-line fallback (the pre-2026-09-23 behavior), used ONLY when there's
// no forward route to walk (route build failed / car off-network). Walks
// OUTWARD from the vehicle in fixed steps and reports how far away the speed
// limit first differs from the current one. Quantized to `step` (20m at the
// default 100m range), so it ticks down 100 -> 80 -> 60 rather than sliding.
static const int kMaxAheadSamples = 8;
static const float kMinAheadStepM = 20.0f;

// Dynamic warning distance based on vehicle speed:
// Minimum 100m. Scales smoothly with vehicle speed (lead time ~14-15s: ~4.5m per km/h).
// e.g.: <=22 km/h -> 100m, 50 km/h -> 225m, 60 km/h -> 270m, 80 km/h -> 360m, 100 km/h -> 450m, 120 km/h -> 540m (capped at 600m).
float computeDynamicWarnDistance(float speedKmh) {
    float d = speedKmh * 4.5f;
    if (d < 100.0f) d = 100.0f; // Minimum 100m
    if (d > 600.0f) d = 600.0f; // Maximum 600m
    return d;
}

static bool findAheadLimitChangeStraight(const GnssSnapshot &gnss, float currentLimitKmh, float &outDistM,
                                         float &outLimitKmh) {
    float maxM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
    float step = maxM / (float)kMaxAheadSamples;
    if (step < kMinAheadStepM) step = kMinAheadStepM;
    for (float d = step; d <= maxM + 0.01f; d += step) {
        MatchCandidate cand;
        if (!matchAheadPoint(gnss, d, cand)) continue; // no confident match at this sample — not evidence of a change
        if ((float)cand.speedLimitKmh != currentLimitKmh) {
            outDistM = d;
            outLimitKmh = (float)cand.speedLimitKmh;
            return true;
        }
    }
    return false;
}

// ROUTE-BASED ahead-limit lookahead (2026-09-23). Walks the forward route from
// the car's arc-length position and reports the first segment whose speed limit
// differs from the current one, with its true remaining distance. Unlike the
// straight-line version above, this follows the actual road through curves and
// junctions, so the "limit ahead" is the one the car will really reach — not
// one on a parallel road the straight projection happened to clip. Falls back
// to the straight-line scan when there's no usable route.
static bool findAheadLimitChange(const GnssSnapshot &gnss, float currentLimitKmh, float &outDistM,
                                  float &outLimitKmh) {
    float maxM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
    if (gRouteCount > 0 && gCarOnRoute) {
        if (routeFindAheadLimitChange(gCarDistM, currentLimitKmh, maxM, outDistM, outLimitKmh)) {
            // A change past an unpassed fork may be on the other branch (e.g.
            // the 40 on an exit ramp while staying on the 80 mainline) — hold it
            // back until the car commits to a branch.
            if (routeBeyondUnpassedFork(gCarDistM + outDistM)) return false;
            return true;
        }
        // Route exists and we walked it fully — no change ahead within the
        // built route. Don't fall through to the straight-line scan: it would
        // project off-road past the route's end and invent a change on some
        // crossing street. No change is the honest answer.
        return false;
    }
    return findAheadLimitChangeStraight(gnss, currentLimitKmh, outDistM, outLimitKmh);
}

// ---------------------------------------------------------------------
// Upcoming speed-camera warning (feature-requested 2026-09-21, "tai du lieu
// ve canh bao giao thong, gom camera cung nhu gioi han toc do"). Scans the
// WHOLE flat CameraPoint[] (map/SdCardManager.cpp's sdMgrGetCameras() — no
// tiling needed, see that array's own comment on why real camera counts
// stay small) rather than doing a tile lookup the way the road matcher
// does — simplest correct approach for what's at most a few thousand
// points, and this only runs once per ~500ms match tick, not per frame.
// Completely independent of runMatch()'s own road-segment result/
// hysteresis state: a camera can be "ahead" whether or not the current
// road segment itself matched.
// User-tunable (2026-09-22, "hieu chinh khoang cach canh bao camera phia
// truoc") — was a fixed 300m constant here, now AppConfig.h's
// cfg.cameraWarnDistM (Settings > Sensors), clamped 100-800m; real
// turn-by-turn nav apps commonly warn 200-500m before a camera, hence that
// default and range.
static const float kCameraBearingToleranceDeg = 60.0f; // how far off dead-ahead a camera can be and still count as "ahead" rather than off to the side/behind (straight-line fallback only)

// ROUTE-BASED camera lookahead (2026-09-23). Projects each camera onto the
// forward route and keeps the NEAREST one AHEAD of the car by arc length, so
// the reported distance is how far you actually drive to reach it (around
// curves, through junctions), not a straight-line range that miscounts on a
// bend. The lateral tolerance keeps a camera on a parallel/adjacent road from
// being picked up. Falls back to the old bearing-cone scan when there's no
// usable route.
// Number shown on the camera card (2026-09-27, bench check against the card's
// data: a camera tagged 60 showed "40"). Source data often has the same camera
// twice a few metres apart, one copy with the limit and one without — and the
// copy without it can be the one that sits on our road. So: the camera's own
// limit; else a same-direction duplicate within 30 m that has one; else the
// current road limit ONLY if it comes from a real sign/tag (never a road-class
// DEFAULT guess); else -1 (card shows the camera icon alone).
static float cameraCardLimit(const CamPt *best, const CamPt *cams, int camCount, const RoadInfoSnapshot &road) {
    if (best->speedLimitKmh > 0) return (float)best->speedLimitKmh;
    const int32_t dLat = (int32_t)(30.0f / 110540.0f * 1e7f);
    float bLat = best->latE7 / 1e7f, bLon = best->lonE7 / 1e7f;
    float bestD = 31.0f, lim = -1.0f;
    for (int i = pointsLatBegin(cams, camCount, best->latE7 - dLat); i < camCount && cams[i].latE7 <= best->latE7 + dLat; i++) {
        const CamPt &c = cams[i];
        if (&c == best || c.speedLimitKmh <= 0) continue;
        if (best->directionDeg != 0xFFFF && c.directionDeg != 0xFFFF &&
            angularDiffDeg((float)best->directionDeg, (float)c.directionDeg) > 45.0f)
            continue;
        Vec2 v = toLocalMeters(c.latE7 / 1e7f, c.lonE7 / 1e7f, bLat, bLon);
        float d = sqrtf(v.x * v.x + v.y * v.y);
        if (d < bestD) { bestD = d; lim = (float)c.speedLimitKmh; }
    }
    if (lim > 0) return lim;
    if (road.valid && road.speedLimitKmh > 0 && road.source != SPEED_SOURCE_DEFAULT && road.source != SPEED_SOURCE_UNKNOWN)
        return road.speedLimitKmh;
    return -1.0f;
}

static void matchCameraAheadRoute(const GnssSnapshot &gnss, const RoadInfoSnapshot &roadMatch, RoadInfoSnapshot &out) {
    const CamPt *cams;
    int camCount;
    sdMgrGetCameras(&cams, &camCount);
    if (camCount == 0) return;

    float warnDistM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
    float bestAheadDistM = 1e9f;
    const CamPt *best = NULL;

    // Bounding-box prefilter around the car, then project onto the route.
    float dLatMax = (warnDistM + kRouteLateralTolM) / 110540.0f + 0.0005f;
    int32_t bandLo = (int32_t)((gnss.latDeg - (dLatMax)) * 1e7f), bandHi = (int32_t)((gnss.latDeg + (dLatMax)) * 1e7f);
    for (int i = pointsLatBegin(cams, camCount, bandLo); i < camCount && cams[i].latE7 <= bandHi; i++) {
        float camLat = cams[i].latE7 / 1e7f, camLon = cams[i].lonE7 / 1e7f;
        if (fabsf(camLat - gnss.latDeg) > dLatMax) continue;
        Vec2 p = toLocalMeters(camLat, camLon, gnss.latDeg, gnss.lonDeg);
        if (sqrtf(p.x * p.x + p.y * p.y) > warnDistM + kRouteLateralTolM) continue;

        // Ownership, not just proximity: a camera on the parallel/opposite/
        // lower road is rejected. NO direction filter for cameras — their
        // stored directionDeg is effectively random in the data (measured
        // 2026-09-26: 25% along travel, 23% opposite, 52% sideways on one-way
        // roads). Not deferred past a fork either: a missed camera costs more
        // than an early one.
        float camArc;
        if (!routeOwnsItem(0x80000000u | (uint32_t)i, cams[i].latE7, cams[i].lonE7, &camArc, NULL)) continue;
        float aheadDist = camArc - gCarDistM;     // remaining route distance to the camera
        if (aheadDist < 0) continue;             // behind the car
        if (aheadDist > warnDistM) continue;     // beyond the warn horizon
        if (aheadDist < bestAheadDistM) {
            bestAheadDistM = aheadDist;
            best = &cams[i];
        }
    }
    if (!best) return;

    out.cameraAheadValid = true;
    out.cameraAheadDistanceM = bestAheadDistM;
    out.cameraSpeedLimitKmh = cameraCardLimit(best, cams, camCount, roadMatch);
}

static void matchCameraAheadStraight(const GnssSnapshot &gnss, const RoadInfoSnapshot &roadMatch, RoadInfoSnapshot &out) {
    const CamPt *cams;
    int camCount;
    sdMgrGetCameras(&cams, &camCount);
    if (camCount == 0) return;

    float warnDistM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
    float bestDistM = 1e9f;
    const CamPt *best = NULL;
    int32_t bandLo = (int32_t)((gnss.latDeg - (warnDistM / 110540.0f + 0.0005f)) * 1e7f), bandHi = (int32_t)((gnss.latDeg + (warnDistM / 110540.0f + 0.0005f)) * 1e7f);
    for (int i = pointsLatBegin(cams, camCount, bandLo); i < camCount && cams[i].latE7 <= bandHi; i++) {
        float camLat = cams[i].latE7 / 1e7f, camLon = cams[i].lonE7 / 1e7f;
        Vec2 p = toLocalMeters(camLat, camLon, gnss.latDeg, gnss.lonDeg); // camera's offset FROM the fix
        float distM = sqrtf(p.x * p.x + p.y * p.y);
        if (distM > warnDistM || distM >= bestDistM) continue; // already farther than the current best — skip the trig below for it
        float bearingToCameraDeg = atan2f(p.x, p.y) * (180.0f / (float)M_PI); // 0=north/clockwise, same convention as headingDeg
        if (bearingToCameraDeg < 0) bearingToCameraDeg += 360.0f;
        if (angularDiffDeg(gnss.headingDeg, bearingToCameraDeg) > kCameraBearingToleranceDeg) continue;
        bestDistM = distM;
        best = &cams[i];
    }
    if (!best) return;

    out.cameraAheadValid = true;
    out.cameraAheadDistanceM = bestDistM;
    out.cameraSpeedLimitKmh = cameraCardLimit(best, cams, camCount, roadMatch);
}

static void matchCameraAhead(const GnssSnapshot &gnss, const RoadInfoSnapshot &roadMatch, RoadInfoSnapshot &out) {
    if (!gnss.fix) return;
    // A usable route makes "ahead" mean along-the-road; without one we need a
    // trustworthy heading for the straight-line bearing cone.
    if (gRouteCount > 0 && gCarOnRoute) {
        matchCameraAheadRoute(gnss, roadMatch, out);
    } else if (gnss.headingValid) {
        matchCameraAheadStraight(gnss, roadMatch, out);
    }
}

// kSignWarnDistanceM now lives in SpeedLimitManager.h — ui/Dashboard.cpp
// needs the same number for its distance-reactive alert card; see the
// header for why it's shared rather than duplicated.
static const float kSignBearingToleranceDeg = 50.0f; // straight-line fallback only

// Shared tail for both sign-lookahead variants (route-based and straight-line
// fallback): takes the nearest-ahead sign of each type that the scan found and
// writes it into the snapshot, remembers the last speed-limit sign actually
// passed (for the no-tag fallback below), and applies the fallback current-limit
// logic. Factored out 2026-09-23 so the route rewrite and the old bearing-cone
// path can't drift apart in how they populate the same fields.
static void fillSignResults(RoadInfoSnapshot &out, const SignPt *bestSpeedSign, float bestSpeedDistM,
                            const SignPt *bestResident, float bestResidentDistM,
                            const SignPt *bestNoOvertake, float bestNoOvertakeDistM,
                            const SignPt *bestToll, float bestTollDistM, const SignPt *bestLight,
                            float bestLightDistM, const SignPt *bestDanger, float bestDangerDistM,
                            const SignPt *closestSign, float closestSignDistM) {
    static float sLastPassedSpeedLimit = -1.0f;
    static uint32_t sLastPassedSpeedLimitMs = 0;

    if (bestSpeedSign) {
        out.aheadLimitValid = true;
        out.aheadSpeedLimitKmh = (float)bestSpeedSign->speedLimitKmh;
        out.aheadDistanceM = bestSpeedDistM;

        if (bestSpeedDistM < 35.0f) {
            sLastPassedSpeedLimit = (float)bestSpeedSign->speedLimitKmh;
            sLastPassedSpeedLimitMs = millis();
        }
    }

    if (bestResident) {
        out.residentAreaAheadValid = true;
        out.residentAreaAheadDistM = bestResidentDistM;
        out.residentAreaIsStart = (bestResident->subType == 0);
    }
    if (bestNoOvertake) {
        out.noOvertakingAheadValid = true;
        out.noOvertakingAheadDistM = bestNoOvertakeDistM;
        out.noOvertakingIsStart = (bestNoOvertake->subType == 0);
    }
    if (bestToll) {
        out.tollBoothAheadValid = true;
        out.tollBoothAheadDistM = bestTollDistM;
    }
    if (bestLight) {
        out.trafficLightAheadValid = true;
        out.trafficLightAheadDistM = bestLightDistM;
    }
    if (bestDanger) {
        out.dangerAheadValid = true;
        out.dangerAheadDistM = bestDangerDistM;
    }
    if (closestSign) {
        out.nextSignType = closestSign->signType;
        out.nextSignDistanceM = closestSignDistM;
        out.nextSignSpeedLimit = closestSign->speedLimitKmh;
    }

    // Fallback speed limit when OSM segment has no maxspeed tag
    if (!out.valid) {
        if (sLastPassedSpeedLimit > 0 && (millis() - sLastPassedSpeedLimitMs < 300000)) {
            // Use recently passed speed limit sign (active within last 5 minutes)
            out.valid = true;
            out.speedLimitKmh = sLastPassedSpeedLimit;
            out.source = SPEED_SOURCE_OSM_MAXSPEED;
        } else if (bestSpeedSign && bestSpeedDistM < 120.0f) {
            // Speed limit sign right ahead
            out.valid = true;
            out.speedLimitKmh = (float)bestSpeedSign->speedLimitKmh;
            out.source = SPEED_SOURCE_OSM_MAXSPEED;
        } else if (out.residentAreaAheadValid && out.residentAreaIsStart) {
            // In residential area (default 50 km/h in Vietnam)
            out.valid = true;
            out.speedLimitKmh = 50.0f;
            out.source = SPEED_SOURCE_DEFAULT;
        }
    }
}

// ROUTE-BASED sign lookahead (2026-09-23) — the fix for "biểu tượng cảnh báo
// tốc độ, cảnh báo giao thông tiếp theo chưa chính xác". Projects each sign
// onto the forward route and measures its remaining ARC-LENGTH distance ahead
// of the car, then keeps the NEAREST one of each type by that route distance.
// Two things this gets right that the old bearing-cone scan did not:
//   * The "next" sign is the next one you actually REACH along the road, even
//     around a bend or past a junction — a straight-line cone either missed a
//     sign just off-axis or picked one on a parallel road that happened to be
//     within the cone.
//   * A sign's own orientation tag (directionDeg) is compared against the
//     ROUTE's local heading at that sign, not the car's raw heading, so a sign
//     on a curve is judged by the road direction there.
// Falls back to the bearing-cone scan when there's no usable route.
// ---- Legal default limits (2026-09-27, "bien het han che toc do thi doan
// duong tiep theo se han che toc do theo quy dinh theo luat, hoac theo so lieu
// ma du lieu co"). Thong tu 38/2024/TT-BGTVT (from 01/01/2025), cars and
// light trucks, no speed sign in force:
//   inside a residential area  : 60 divided road / one-way >=2 lanes, else 50
//   outside a residential area : 90 divided road / one-way >=2 lanes, else 80
// "Divided" is approximated by a one-way segment of a major/main road class
// (OSM maps dual carriageways as two one-way ways; small one-way streets are
// usually one lane). Expressways always carry their own signs.
static const RouteSeg *routeSegAtArc(float arc) {
    for (int i = 0; i < gRouteCount; i++) {
        const RouteSeg &r = gRoute.seg(i);
        if (arc >= r.startDistM && arc <= r.startDistM + r.lenM) return &r;
    }
    return NULL;
}
static float legalDefaultKmh(bool residential, const RouteSeg *seg) {
    bool divided = seg && (seg->flags & SEGFLAG_ONEWAY) && (seg->roadClass == 1 || seg->roadClass == 2);
    return residential ? (divided ? 60.0f : 50.0f) : (divided ? 90.0f : 80.0f);
}
// Limit in force just past `arc` when no numbered speed sign applies there:
// the road's real tag if it has one; else the law (when we know whether we're
// in a residential area); else the data's own value for that road.
static float limitWithoutSignAt(float arc, int residentialState, uint8_t *outSrc) {
    const RouteSeg *seg = routeSegAtArc(arc + 1.0f);
    if (seg && seg->speedLimitKmh > 0 && seg->source != SPEED_SOURCE_DEFAULT && seg->source != SPEED_SOURCE_UNKNOWN) {
        *outSrc = SPEED_SOURCE_OSM_MAXSPEED;
        return (float)seg->speedLimitKmh;
    }
    *outSrc = SPEED_SOURCE_DEFAULT;
    if (residentialState >= 0) return legalDefaultKmh(residentialState == 1, seg);
    if (seg && seg->speedLimitKmh > 0) return (float)seg->speedLimitKmh;
    return legalDefaultKmh(false, seg);
}
static void setSignOverride(float kmh, uint8_t src, const char *why) {
    if (kmh != gSignOverrideKmh || src != gSignOverrideSrc)
        Serial.printf("[map] %s -> current limit %.0f (%s)\n", why, (double)kmh, speedSourceStr(src));
    gSignOverrideKmh = kmh;
    gSignOverrideSrc = src;
    gSignOverrideMs = millis();
}

static void matchSignsAheadRoute(const GnssSnapshot &gnss, RoadInfoSnapshot &out) {
    const SignPt *signs;
    int signCount;
    sdMgrGetSigns(&signs, &signCount);
    if (signCount == 0) return;

    float signWarnDistM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
    float dLatMax = (signWarnDistM + kRouteLateralTolM) / 110540.0f + 0.0005f;

    float closestSignDistM = 1e9f;
    const SignPt *closestSign = NULL;
    float bestSpeedDistM = 1e9f;  const SignPt *bestSpeedSign = NULL;
    float bestEndDistM = 1e9f, bestEndArc = 0; const SignPt *bestEnd = NULL; // speed sign with value 0 = end of restriction
    float bestResidentDistM = 1e9f; const SignPt *bestResident = NULL; float bestResidentArc = 0;
    float bestNoOvertakeDistM = 1e9f; const SignPt *bestNoOvertake = NULL;
    float bestTollDistM = 1e9f;   const SignPt *bestToll = NULL;
    float bestLightDistM = 1e9f;  const SignPt *bestLight = NULL;
    float bestDangerDistM = 1e9f; const SignPt *bestDanger = NULL;

    int32_t bandLo = (int32_t)((gnss.latDeg - (dLatMax)) * 1e7f), bandHi = (int32_t)((gnss.latDeg + (dLatMax)) * 1e7f);
    for (int i = pointsLatBegin(signs, signCount, bandLo); i < signCount && signs[i].latE7 <= bandHi; i++) {
        float signLat = signs[i].latE7 / 1e7f;
        float signLon = signs[i].lonE7 / 1e7f;
        Vec2 p = toLocalMeters(signLat, signLon, gnss.latDeg, gnss.lonDeg);
        if (sqrtf(p.x * p.x + p.y * p.y) > signWarnDistM + kRouteLateralTolM) continue;

        float arc, routeHeading;
        if (!routeOwnsItem((uint32_t)i, signs[i].latE7, signs[i].lonE7, &arc, &routeHeading)) continue;
        float aheadDist = arc - gCarDistM;   // remaining route distance to the sign
        if (aheadDist < 0 || aheadDist > signWarnDistM) continue; // behind, or beyond horizon
        if (routeBeyondUnpassedFork(arc)) continue; // may be on the branch we won't take

        // Sign's own facing direction, judged against the ROAD heading at the
        // sign (routeHeading), not the car's raw heading — correct around curves.
        if (signs[i].directionDeg != 0xFFFF) {
            if (angularDiffDeg(routeHeading, (float)signs[i].directionDeg) > 60.0f) continue;
        }

        if (aheadDist < closestSignDistM) { closestSignDistM = aheadDist; closestSign = &signs[i]; }

        uint8_t t = signs[i].signType;
        if (t == SIGN_TYPE_SPEED_LIMIT && signs[i].speedLimitKmh > 0 && aheadDist < bestSpeedDistM) {
            bestSpeedDistM = aheadDist; bestSpeedSign = &signs[i];
        } else if (t == SIGN_TYPE_SPEED_LIMIT && signs[i].speedLimitKmh == 0 && aheadDist < bestEndDistM) {
            bestEndDistM = aheadDist; bestEnd = &signs[i]; bestEndArc = arc;
        } else if (t == SIGN_TYPE_RESIDENT_AREA && aheadDist < bestResidentDistM) {
            bestResidentDistM = aheadDist; bestResident = &signs[i]; bestResidentArc = arc;
        } else if (t == SIGN_TYPE_NO_OVERTAKING && aheadDist < bestNoOvertakeDistM) {
            bestNoOvertakeDistM = aheadDist; bestNoOvertake = &signs[i];
        } else if (t == SIGN_TYPE_TOLL_BOOTH && aheadDist < bestTollDistM) {
            bestTollDistM = aheadDist; bestToll = &signs[i];
        } else if (t == SIGN_TYPE_TRAFFIC_LIGHT && aheadDist < bestLightDistM) {
            bestLightDistM = aheadDist; bestLight = &signs[i];
        } else if (t == SIGN_TYPE_DANGER_OTHER && aheadDist < bestDangerDistM) {
            bestDangerDistM = aheadDist; bestDanger = &signs[i];
        }
    }
    // Passing signs (within 35 m ahead), applied in road order so the LAST one
    // reached wins: residential start/end sets the area state (and, with no
    // numbered sign, the legal limit); an end-of-restriction sign returns to
    // the tag/law; a numbered speed sign sets its number.
    // "End of restriction" followed by a new numbered limit (bench: 32 m apart)
    // are one change: the number applies directly, no brief legal-default value
    // between them (was a visible 40 -> 80 flicker). A numbered sign still AHEAD
    // of the end sign also defers the end sign until it has been passed.
    if (bestEnd && bestSpeedSign && bestSpeedDistM - bestEndDistM < 50.0f) bestEnd = NULL;
    struct Pass { float d; int kind; };
    Pass ps[3];
    int np = 0;
    if (bestResident && bestResidentDistM < 35.0f) ps[np++] = {bestResidentDistM, 0};
    if (bestEnd && bestEndDistM < 35.0f) ps[np++] = {bestEndDistM, 1};
    if (bestSpeedSign && bestSpeedDistM < 35.0f && bestSpeedSign->speedLimitKmh >= 5) ps[np++] = {bestSpeedDistM, 2};
    for (int a = 0; a < np; a++)
        for (int b = a + 1; b < np; b++)
            if (ps[b].d < ps[a].d) { Pass t = ps[a]; ps[a] = ps[b]; ps[b] = t; }
    for (int k = 0; k < np; k++) {
        uint8_t src;
        if (ps[k].kind == 0) {
            int st = bestResident->subType == 0 ? 1 : 0;
            if (st != gResidentialState)
                Serial.printf("[map] residential area %s\n", st ? "START (R.420)" : "END (R.421)");
            gResidentialState = st;
            float v = limitWithoutSignAt(bestResidentArc, gResidentialState, &src);
            setSignOverride(v, src, st ? "entering residential area" : "leaving residential area");
        } else if (ps[k].kind == 1) {
            float v = limitWithoutSignAt(bestEndArc, gResidentialState, &src);
            setSignOverride(v, src, "end-of-restriction sign");
        } else {
            setSignOverride((float)bestSpeedSign->speedLimitKmh, SPEED_SOURCE_OSM_MAXSPEED, "speed sign");
        }
    }
    fillSignResults(out, bestSpeedSign, bestSpeedDistM, bestResident, bestResidentDistM, bestNoOvertake,
                    bestNoOvertakeDistM, bestToll, bestTollDistM, bestLight, bestLightDistM, bestDanger,
                    bestDangerDistM, closestSign, closestSignDistM);
    // An end-of-restriction sign ahead (nearer than any numbered sign): show the
    // limit that will apply after it on the limit-ahead card.
    if (bestEnd && bestEndDistM < bestSpeedDistM) {
        uint8_t src;
        float v = limitWithoutSignAt(bestEndArc, gResidentialState, &src);
        if (!out.valid || v != out.speedLimitKmh) {
            out.aheadLimitValid = true;
            out.aheadSpeedLimitKmh = v;
            out.aheadDistanceM = bestEndDistM;
        }
    }
}

static void matchSignsAheadStraight(const GnssSnapshot &gnss, RoadInfoSnapshot &out) {
    const SignPt *signs;
    int signCount;
    sdMgrGetSigns(&signs, &signCount);
    if (signCount == 0) return;

    float closestSignDistM = 1e9f;
    const SignPt *closestSign = NULL;

    float bestSpeedDistM = 1e9f;
    const SignPt *bestSpeedSign = NULL;

    float bestResidentDistM = 1e9f;
    const SignPt *bestResident = NULL;

    float bestNoOvertakeDistM = 1e9f;
    const SignPt *bestNoOvertake = NULL;

    float bestTollDistM = 1e9f;
    const SignPt *bestToll = NULL;

    float bestLightDistM = 1e9f;
    const SignPt *bestLight = NULL;

    float bestDangerDistM = 1e9f;
    const SignPt *bestDanger = NULL;

    int32_t bandLo = (int32_t)((gnss.latDeg - (0.005f)) * 1e7f), bandHi = (int32_t)((gnss.latDeg + (0.005f)) * 1e7f);
    for (int i = pointsLatBegin(signs, signCount, bandLo); i < signCount && signs[i].latE7 <= bandHi; i++) {
        float signLat = signs[i].latE7 / 1e7f;
        float signLon = signs[i].lonE7 / 1e7f;

        // Fast bounding box check (lat/lon)
        float dLat = fabsf(signLat - gnss.latDeg);
        if (dLat > 0.005f) continue; // ~550m

        Vec2 p = toLocalMeters(signLat, signLon, gnss.latDeg, gnss.lonDeg);
        float distM = sqrtf(p.x * p.x + p.y * p.y);
        float signWarnDistM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
        if (distM > signWarnDistM) continue;

        // Check bearing to sign from car
        float bearingToSignDeg = atan2f(p.x, p.y) * (180.0f / (float)M_PI);
        if (bearingToSignDeg < 0) bearingToSignDeg += 360.0f;
        if (angularDiffDeg(gnss.headingDeg, bearingToSignDeg) > kSignBearingToleranceDeg) continue;

        // Also check sign's own orientation if tagged
        if (signs[i].directionDeg != 0xFFFF) {
            if (angularDiffDeg(gnss.headingDeg, signs[i].directionDeg) > 60.0f) {
                continue; // Sign faces away or perpendicular to car's travel lane
            }
        }

        if (distM < closestSignDistM) {
            closestSignDistM = distM;
            closestSign = &signs[i];
        }

        // Category sorting
        if (signs[i].signType == SIGN_TYPE_SPEED_LIMIT && signs[i].speedLimitKmh > 0 && distM < bestSpeedDistM) {
            bestSpeedDistM = distM;
            bestSpeedSign = &signs[i];
        } else if (signs[i].signType == SIGN_TYPE_RESIDENT_AREA && distM < bestResidentDistM) {
            bestResidentDistM = distM;
            bestResident = &signs[i];
        } else if (signs[i].signType == SIGN_TYPE_NO_OVERTAKING && distM < bestNoOvertakeDistM) {
            bestNoOvertakeDistM = distM;
            bestNoOvertake = &signs[i];
        } else if (signs[i].signType == SIGN_TYPE_TOLL_BOOTH && distM < bestTollDistM) {
            bestTollDistM = distM;
            bestToll = &signs[i];
        } else if (signs[i].signType == SIGN_TYPE_TRAFFIC_LIGHT && distM < bestLightDistM) {
            bestLightDistM = distM;
            bestLight = &signs[i];
        } else if (signs[i].signType == SIGN_TYPE_DANGER_OTHER && distM < bestDangerDistM) {
            bestDangerDistM = distM;
            bestDanger = &signs[i];
        }
    }
    fillSignResults(out, bestSpeedSign, bestSpeedDistM, bestResident, bestResidentDistM, bestNoOvertake,
                    bestNoOvertakeDistM, bestToll, bestTollDistM, bestLight, bestLightDistM, bestDanger,
                    bestDangerDistM, closestSign, closestSignDistM);
}

static void matchSignsAhead(const GnssSnapshot &gnss, RoadInfoSnapshot &out) {
    if (!gnss.fix) return;
    if (gRouteCount > 0 && gCarOnRoute) {
        matchSignsAheadRoute(gnss, out);
    } else if (gnss.headingValid) {
        matchSignsAheadStraight(gnss, out);
    }
}

// ---------------------------------------------------------------------
// Nearby-markers query for map/MapRenderer.cpp's live background map — see
// SpeedLimitManager.h's own declaration comment. Plain linear scan, not a
// tile lookup: cameras.bin/signs.bin are already flat, untiled, whole-in-RAM
// arrays (SdCardManager.h), same reasoning matchCameraAhead()/
// matchSignsAhead() above already scan them directly rather than through
// any tile cache. `out[].kind` is a raw TrafficSignType value
// (SpeedMapFormat.h) — SIGN_TYPE_CAMERA for camera points, signs[i].signType
// for sign points — left for MapRenderer.cpp to translate into its own
// MapMarkerKind, so this header doesn't need to know that enum exists.
bool speedLimitManagerGetNearbyMarkers(float lat, float lon, float radiusM, NearbyMarkerRaw *out, int maxOut,
                                        int *outCount) {
    *outCount = 0;
    if (!gMapLoaded) return false;

    float radiusSqM = radiusM * radiusM;
    float cosLat = cosf(lat * (float)M_PI / 180.0f);
    int n = 0;

    const CamPt *cams;
    int camCount;
    sdMgrGetCameras(&cams, &camCount);
    int32_t bandLo = (int32_t)((lat - radiusM / 110540.0f) * 1e7f), bandHi = (int32_t)((lat + radiusM / 110540.0f) * 1e7f);
    for (int i = pointsLatBegin(cams, camCount, bandLo); i < camCount && cams[i].latE7 <= bandHi && n < maxOut; i++) {
        float camLat = cams[i].latE7 / 1e7f, camLon = cams[i].lonE7 / 1e7f;
        float dxM = (camLon - lon) * 111320.0f * cosLat;
        float dyM = (camLat - lat) * 110540.0f;
        if (dxM * dxM + dyM * dyM > radiusSqM) continue;
        out[n].lat = camLat;
        out[n].lon = camLon;
        out[n].kind = SIGN_TYPE_CAMERA;
        n++;
    }

    const SignPt *signs;
    int signCount;
    sdMgrGetSigns(&signs, &signCount);
    for (int i = pointsLatBegin(signs, signCount, bandLo); i < signCount && signs[i].latE7 <= bandHi && n < maxOut; i++) {
        float signLat = signs[i].latE7 / 1e7f, signLon = signs[i].lonE7 / 1e7f;
        float dxM = (signLon - lon) * 111320.0f * cosLat;
        float dyM = (signLat - lat) * 110540.0f;
        if (dxM * dxM + dyM * dyM > radiusSqM) continue;
        out[n].lat = signLat;
        out[n].lon = signLon;
        out[n].kind = signs[i].signType;
        n++;
    }

    *outCount = n;
    return true;
}

// ---------------------------------------------------------------------
static void runSelfTest() {
    if (gMetadata.testRoadId == 0) {
        Serial.println("[map] database has no self-test point (testRoadId=0) — skipping self-test");
        return;
    }
    GnssSnapshot fake;
    fake.fix = true;
    fake.latDeg = gMetadata.testLatE7 / 1e7f;
    fake.lonDeg = gMetadata.testLonE7 / 1e7f;
    fake.headingValid = true;
    fake.headingDeg = (float)gMetadata.testHeadingDeg;

    RoadInfoSnapshot result;
    runMatch(fake, result);
    bool pass = result.valid && result.roadId == gMetadata.testRoadId;
    Serial.printf("[map] self-test %s — expected road=%lu got road=%lu limit=%.0fkm/h confidence=%.2f\n",
                  pass ? "PASS" : "FAIL", (unsigned long)gMetadata.testRoadId, (unsigned long)result.roadId,
                  (double)result.speedLimitKmh, (double)result.confidence);

    // Reset continuity state so the synthetic self-test point doesn't bias
    // the very first real match via the continuity bonus.
    currentRoadId = 0;
    currentConfidence = 0;
    lastValidMs = 0;
    lastPublished = RoadInfoSnapshot();
    // ...and the forward-route + dead-reckon state too — runMatch() built a
    // route off the synthetic test point; leaving it would seed the first real
    // match with a route anchored somewhere the car isn't.
    resetRouteState();
    gFixLostMs = 0;
    gLastDeadReckonMs = 0;
    gLastKnownSpeedKmh = 0;
}

static void speedLimitTaskFn(void *) {
    allocateTileCache();
    RoadInfoSnapshot ri;
    if (!sdMgrMount()) {
        Serial.println("[map] SD card/database unavailable — speed limit will report UNKNOWN (mapLoaded=false)");
    } else {
        sdMgrGetMetadata(gMetadata);
        ri.mapLoaded = true;
        gMapLoaded = true;
        runSelfTest();
    }
    lastPublished = ri;
    roadInfoPublish(ri);

    esp_task_wdt_add(NULL);
    uint32_t lastDebugMs = 0;
    for (;;) {
        if (!ri.mapLoaded) {
            static uint32_t lastMountRetryMs = 0;
            uint32_t nowMount = millis();
            if (nowMount - lastMountRetryMs > 3000) {
                lastMountRetryMs = nowMount;
                if (sdMgrMount()) {
                    sdMgrGetMetadata(gMetadata);
                    ri.mapLoaded = true;
                    gMapLoaded = true;
                    runSelfTest();
                    Serial.println("[map] SD card inserted and mounted successfully!");
                }
            }
        }
        if (ri.mapLoaded) {
#ifdef VIETHUD_P4
            // P4 (800x480): the 2 Hz map refresh the matcher's 500 ms loop gave
            // made the map jump ~8 m at 60 km/h — read as flicker on the big
            // panel (2026-09-29). Matching stays at 500 ms; in between, this
            // same task (it owns the tile cache) refreshes only the map layer
            // every 100 ms.
            static uint32_t sLastMatchMs = 0;
            if (millis() - sLastMatchMs < 500) {
                mapRendererUpdate(gnssSnapshot());
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            sLastMatchMs = millis();
#endif
            GnssSnapshot gnss = gnssSnapshot();
            RoadInfoSnapshot out;
            uint32_t tTick0 = micros();
            runMatch(gnss, out);
            uint32_t tMatch = micros() - tTick0;
            if (gSignOverrideKmh > 0) {
                if (millis() - gSignOverrideMs > kSignOverrideMaxMs) clearSignOverride("expired");
                else if (!gCarOnRoute) clearSignOverride("off route");
                else if (out.valid || gnss.fix) {
                    out.valid = true;
                    out.speedLimitKmh = gSignOverrideKmh;
                    out.source = gSignOverrideSrc; // OSM_MAXSPEED = a real sign; DEFAULT = legal default after an end sign
                }
            }

            // Ahead-lookahead (see findAheadLimitChange's own comment) —
            // runs while actually moving and EITHER we have a trustworthy
            // heading OR we're sitting on a built forward route. The route is
            // the better source (it follows the road, not a straight line),
            // and accepting it without a heading matters right after a GPS
            // re-acquisition, when course-over-ground is still settling but the
            // car's position already snaps back onto the persisted route — so
            // the limit/warnings come back immediately instead of staying
            // UNKNOWN until the heading stabilizes. Only surfaced when it's a
            // genuinely DIFFERENT limit — re-confirming the same number ahead
            // isn't a "change" worth a driver's attention.
            bool routeUsable = (gRouteCount > 0 && gCarOnRoute);
            bool moving = gnss.fix && gnss.egoSpeedKmh > kGnssMotionThresholdKmh;
            if (moving && (gnss.headingValid || routeUsable) && out.valid) {
                float aheadDistM, aheadLimitKmh;
                if (findAheadLimitChange(gnss, out.speedLimitKmh, aheadDistM, aheadLimitKmh)) {
                    out.aheadLimitValid = true;
                    out.aheadSpeedLimitKmh = aheadLimitKmh;
                    out.aheadDistanceM = aheadDistM; // real remaining distance, counts down as we approach
                }
            }

            // Camera + sign warnings (see matchCameraAhead's own comment) — same
            // "moving with a heading OR on a route" gate as the ahead-limit
            // lookahead above, but deliberately NOT gated on out.valid: a
            // camera/sign is worth knowing about even on a road segment whose
            // own speed limit this firmware doesn't know.
            if (moving && (gnss.headingValid || routeUsable)) {
                matchCameraAhead(gnss, out, out);
                matchSignsAhead(gnss, out);
            }

            // Fallback limit (user-requested 2026-09-24): when we have a real
            // fix but genuinely couldn't resolve a limit (off the mapped
            // network / no route), show cfg.defaultLimitKmh (default 50, VN's
            // baseline urban limit) rather than a blank "--". 0 = off. Marked
            // SPEED_SOURCE_DEFAULT so diagnostics/telemetry can tell it apart
            // from a real matched limit; only applied with an actual fix, never
            // as a fabricated value while GPS is absent.
            if (!out.valid && gnss.fix && cfg.defaultLimitKmh > 0.0f) {
                out.valid = true;
                out.speedLimitKmh = cfg.defaultLimitKmh;
                out.source = SPEED_SOURCE_DEFAULT;
            }

            lastPublished = out;
            roadInfoPublish(out);
            // Live background map (2026-09-22) — rides this same ~500ms loop
            // rather than a new task; mapRendererUpdate() itself gates on real
            // movement/heading delta (see MapRenderer.cpp's own comment) so a
            // stationary or straight-driving car costs nothing extra here.
            {
                uint32_t tMap0 = micros();
                mapRendererUpdate(gnss);
                uint32_t tMap = micros() - tMap0;
                if (tMap > gTickMaxMapUs) gTickMaxMapUs = tMap;
            }
            uint32_t tTick = micros() - tTick0;
            if (tTick > gTickMaxUs) gTickMaxUs = tTick;
            if (tMatch > gTickMaxMatchUs) gTickMaxMatchUs = tMatch;

            // Same 3s periodic-debug pattern as gnss/GNSS.cpp and
            // radar/LD2451.cpp — lat/lon isn't shown anywhere in the UI, so
            // this is the only way to see whether a real fix is actually
            // landing inside the downloaded map region at all (user-reported
            // 2026-09-16, "toa do GPS da len nhung khong hien thi toc do gioi
            // han" — need to see the real numbers to tell "outside the
            // downloaded region" apart from "in region but not matching"
            // apart from a genuine bug).
            uint32_t now = millis();
            // Warning state, 1/s while moving — what the Dashboard is being told
            // (bench verification of camera/sign/limit-ahead warnings).
            static uint32_t sLastWarnLogMs = 0;
            if (gnss.fix && gnss.egoSpeedKmh > kGnssMotionThresholdKmh && now - sLastWarnLogMs >= 1000) {
                sLastWarnLogMs = now;
                Serial.printf("[warn] lat=%.6f lon=%.6f road=%lu lim=%.0f(%s) cam=%s%.0fm/%.0f sign=%u@%.0fm limAhead=%s%.0f@%.0fm "
                              "res=%d nov=%d toll=%d light=%d danger=%d\n",
                              (double)gnss.latDeg, (double)gnss.lonDeg, (unsigned long)out.roadId,
                              (double)out.speedLimitKmh, speedSourceStr(out.source), out.cameraAheadValid ? "" : "-",
                              (double)(out.cameraAheadValid ? out.cameraAheadDistanceM : 0),
                              (double)(out.cameraAheadValid ? out.cameraSpeedLimitKmh : 0), (unsigned)out.nextSignType,
                              (double)(out.nextSignType ? out.nextSignDistanceM : 0), out.aheadLimitValid ? "" : "-",
                              (double)(out.aheadLimitValid ? out.aheadSpeedLimitKmh : 0),
                              (double)(out.aheadLimitValid ? out.aheadDistanceM : 0), out.residentAreaAheadValid,
                              out.noOvertakingAheadValid, out.tollBoothAheadValid, out.trafficLightAheadValid,
                              out.dangerAheadValid);
            }
            if (now - lastDebugMs > 3000) {
                lastDebugMs = now;
                Serial.printf("[map] fix=%d lat=%.6f lon=%.6f valid=%d roadId=%lu conf=%.2f distM=%.1f "
                              "limit=%.0f source=%s track=%s hyp=%d margin=%.1f\n",
                              gnss.fix, (double)gnss.latDeg, (double)gnss.lonDeg, out.valid,
                              (unsigned long)out.roadId, (double)out.confidence, (double)out.matchDistanceM,
                              (double)out.speedLimitKmh, speedSourceStr(out.source), gTrackReason,
                              gTrack.hypothesisCount(), (double)gTrack.margin());
                Serial.printf("[map] tick max %lums (match %lums, map %lums) route=%d segs, %lu tile loads\n",
                              (unsigned long)(gTickMaxUs / 1000), (unsigned long)(gTickMaxMatchUs / 1000),
                              (unsigned long)(gTickMaxMapUs / 1000), gRouteCount, (unsigned long)gTileLoadsWindow);
                gTileLoadsWindow = 0;
                gTickMaxUs = gTickMaxMatchUs = gTickMaxMapUs = 0;
            }
        }
        esp_task_wdt_reset();
        // 500ms matches spec section 21 ("Map Matching: 1-5 Hz", "không cần
        // chạy map matching ở 30 FPS") while there's real work to do; 2s
        // idle once mapLoaded is permanently false (can't become true again
        // without a reboot), same idle-slower reasoning
        // net/WebPortal.cpp's webTaskFn uses for its own OFF-state delay.
#ifdef VIETHUD_P4
        vTaskDelay(pdMS_TO_TICKS(ri.mapLoaded ? 100 : 2000)); // map-only ticks between matches (see top of loop)
#else
        vTaskDelay(pdMS_TO_TICKS(ri.mapLoaded ? 500 : 2000));
#endif
    }
}

// Bumped 4096 -> 8192 on 2026-09-24: the forward-route engine
// (RoutePredictor) added chunky stack locals to this task's call chain —
// runMatch()'s RoadSegment copies, Route::build()'s cand[kMaxNodeCandidates]
// (12*28B) + visited[kMaxSegments] (64*4B), and routeProject()'s loops — on
// top of the SD_MMC File I/O this task already does. The old 4096 stack (only
// ~920 B free even before this) overflowed and the FreeRTOS stack canary
// panicked ("Stack canary watchpoint triggered (speedLimitTask)") in a boot
// loop. 8192 restored a healthy margin FOR THE BENCH/DEMO load (synthetic road,
// ~24 vector lines).
//
// Bumped 8192 -> 16384 on 2026-09-25 after the user reported frequent crashes
// WHILE DRIVING in Hanoi (never on the bench). This task also runs the whole
// live map render — mapRendererUpdate() draws every vector road segment, street
// name and marker in view straight into the canvas from here, and a dense city
// tile holds far more segments than the demo's 24, so real-world call depth +
// LVGL draw scratch is much larger than anything measured at the desk. Stack
// high-water is now logged in main's [mem] block (speedLimitTaskStackFreeBytes)
// so the true in-city margin is finally visible; 16384 matches the loopTask
// stack, which does comparable UI work, and internal RAM has room (~87 KB free).
static TaskHandle_t sSpeedLimitTaskHandle = NULL;
void speedLimitManagerStart() {
    xTaskCreatePinnedToCore(speedLimitTaskFn, "speedLimitTask", 16384, NULL, 1, &sSpeedLimitTaskHandle, 0);
}
size_t speedLimitTaskStackFreeBytes() {
    if (!sSpeedLimitTaskHandle) return 0;
    return uxTaskGetStackHighWaterMark(sSpeedLimitTaskHandle) * sizeof(StackType_t);
}

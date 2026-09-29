#include "MapRenderer.h"
#include "SpeedLimitManager.h" // speedLimitManagerGetNearbySegments/GetNearbyMarkers, RoadSegment, SIGN_TYPE_*
#include "SnapToRoad.h"        // Orthogonal projection onto matched road segment
#include <math.h>

// ---------------------------------------------------------------------
// Canvas geometry — set once at boot (mapRendererSetCanvasSize(), called
// from ui/Dashboard.cpp's buildMapCanvas() before map/SpeedLimitManager.cpp's
// task starts).
//
// Heading-up rewrite (2026-09-24): the vector layer is now projected
// NORTH-UP (heading is NOT baked into the screen coordinates — see
// mapRendererComputeFromSegments()); ui/Dashboard.cpp rotates the whole map
// canvas by -heading via LVGL image rotation so travel points at 12 o'clock
// and the raster background rotates with it. Because that rotating canvas is
// OVERSIZED (a square big enough that no screen corner is ever left blank at
// any rotation), the ego anchor inside the canvas is its CENTER, while the
// on-screen framing (car 2/3 down) is achieved by where Dashboard positions
// the canvas. So the anchor and the reference "minDim" that sets the zoom are
// now supplied explicitly by Dashboard (mapRendererSetGeometry) rather than
// derived from gCanvasW/H — otherwise the oversized canvas would change the
// zoom. mapRendererSetCanvasSize() keeps the old self-derived behavior for any
// caller that doesn't set an explicit geometry.
// ---------------------------------------------------------------------
static int gCanvasW = 480, gCanvasH = 320;
static int gAnchorX = 240, gAnchorY = 213; // ego anchor inside the (possibly oversized) canvas
static int gRefMinDim = 107;               // on-screen minDim that sets pixels-per-meter (independent of canvas size)

void mapRendererSetCanvasSize(int widthPx, int heightPx) {
    gCanvasW = widthPx;
    gCanvasH = heightPx;
    gAnchorX = widthPx / 2;
    gAnchorY = (heightPx * 2) / 3;
    int bottom = heightPx - gAnchorY;
    int m = gAnchorX;
    if (gAnchorY < m) m = gAnchorY;
    if (bottom < m) m = bottom;
    if (m < 1) m = 1;
    gRefMinDim = m;
}

// Explicit geometry for the oversized heading-up canvas (2026-09-24): the
// canvas is `canvasSide` square with the ego at (anchorX,anchorY) = its
// center, but the zoom stays framed to the visible screen via `refMinDim`
// (the on-screen min of anchorX / anchorY / screenH-anchorY). Dashboard
// computes these from the real screen size and calls this instead of
// mapRendererSetCanvasSize().
void mapRendererSetGeometry(int canvasSide, int anchorX, int anchorY, int refMinDim) {
    gCanvasW = canvasSide;
    gCanvasH = canvasSide;
    gAnchorX = anchorX;
    gAnchorY = anchorY;
    gRefMinDim = refMinDim < 1 ? 1 : refMinDim;
}

static float gZoomMult = 1.0f;
void mapRendererSetZoomMultiplier(float mult) {
    if (mult < 0.5f) mult = 0.5f;
    if (mult > 4.0f) mult = 4.0f;
    gZoomMult = mult;
}

float mapRendererZoomRadiusM(float speedKmh) {
    float base;
    if (speedKmh < 40.0f) base = 240.0f; // Calibrated to match 2.0x raster tile scale (~2.2m/px)
    else if (speedKmh <= 80.0f) base = 450.0f;
    else base = 750.0f;
    return base / gZoomMult; // manual digital zoom (Dashboard) folded in so raster + vector stay matched
}

static float computeScale(float radiusM) {
    // Zoom is framed to the VISIBLE screen (gRefMinDim), not the oversized
    // canvas — see the geometry comment above. Kept as pixels-per-meter.
    return (float)gRefMinDim / radiusM;
}

// North-up projection (2026-09-24): the map is drawn north-up here and the
// whole canvas is rotated to heading-up by ui/Dashboard.cpp (LVGL image
// rotation). So `headingDeg` is passed as 0 by the live path — the rotation is
// no longer baked in — but the parameter is kept so demo/other callers could
// still bake a rotation if ever needed.
//   dx = east offset (m), dy = north offset (m) from ego position
//   screenX = anchorX + (dx*cos(h) - dy*sin(h)) * scale
//   screenY = anchorY - (dx*sin(h) + dy*cos(h)) * scale   (h=0 => north-up)
static void projectToScreen(float lat, float lon, float egoLat, float egoLon, float headingDeg, float scale,
                             int16_t &outX, int16_t &outY) {
    float dx = (lon - egoLon) * 111320.0f * cosf(egoLat * (float)M_PI / 180.0f);
    float dy = (lat - egoLat) * 110540.0f;
    float h = headingDeg * (float)M_PI / 180.0f;
    float sx = dx * cosf(h) - dy * sinf(h);
    float sy = -(dx * sinf(h) + dy * cosf(h));

    float px = gAnchorX + sx * scale;
    float py = gAnchorY + sy * scale;

    if (px < -32000.0f) px = -32000.0f;
    else if (px > 32000.0f) px = 32000.0f;
    if (py < -32000.0f) py = -32000.0f;
    else if (py > 32000.0f) py = 32000.0f;
    outX = (int16_t)px;
    outY = (int16_t)py;
}

static uint8_t translateSignTypeToMarkerKind(uint8_t signType) {
    switch (signType) {
        case SIGN_TYPE_CAMERA: return MAP_MARKER_CAMERA;
        case SIGN_TYPE_RESIDENT_AREA: return MAP_MARKER_RESIDENT_AREA;
        case SIGN_TYPE_NO_OVERTAKING: return MAP_MARKER_NO_OVERTAKING;
        case SIGN_TYPE_TRAFFIC_LIGHT: return MAP_MARKER_TRAFFIC_LIGHT;
        case SIGN_TYPE_TOLL_BOOTH: return MAP_MARKER_TOLL_BOOTH;
        default: return MAP_MARKER_OTHER;
    }
}

static uint32_t gGeneration = 0;

void mapRendererComputeFromSegments(const GnssSnapshot &gnss, const RoadSegment *segs, int segCount) {
    static MapViewSnapshot v;
    v.valid = gnss.fix;
    if (!v.valid) {
        mapViewPublish(v);
        return;
    }

    float radiusM = mapRendererZoomRadiusM(gnss.egoSpeedKmh);
    v.zoomRadiusM = radiusM;
    float scale = computeScale(radiusM);

    float rawHeading = gnss.headingValid ? gnss.headingDeg : 0.0f;

    // Retrieve live road-match result. Deliberately keyed on roadId, NOT
    // road.valid — core/SharedState.h's own RoadInfoSnapshot comment
    // documents valid as "a segment WITH A KNOWN SPEED LIMIT", while
    // roadId/confidence stay meaningful even when a road is confidently
    // matched but just has no speed-limit tag (common off the classified
    // road network). Gating snap-to-road/the current-road highlight on
    // .valid meant both silently stopped working on any untagged road, with
    // nothing wrong about the GPS/heading match itself — found 2026-09-22
    // reviewing the whole matching pipeline for the junction/overpass bug.
    RoadInfoSnapshot road = roadInfoSnapshot();
    const RoadSegment *matchedSeg = nullptr;
    if (road.roadId != 0 && segs) {
        for (int i = 0; i < segCount; i++) {
            if (segs[i].id == road.roadId) {
                matchedSeg = &segs[i];
                break;
            }
        }
    }

    // Snap-to-Road: Center the map around the vehicle's snapped coordinate
    // on the road centerline, eliminating GPS drift and ensuring the vehicle
    // icon is always positioned precisely on the road.
    SnappedPosition snap = SnapToRoad::instance().update(
        gnss.latDeg, gnss.lonDeg, rawHeading, gnss.egoSpeedKmh,
        (matchedSeg != nullptr), matchedSeg
    );

    float egoLat = snap.latDeg;
    float egoLon = snap.lonDeg;
    float egoHeading = snap.headingDeg;
    // North-up projection: draw the map north-up here, publish the heading, and
    // let ui/Dashboard.cpp rotate the whole canvas to heading-up. So every
    // projectToScreen below uses heading 0.
    const float projHeading = 0.0f;
    v.headingUpDeg = egoHeading;
    v.egoLatDeg = egoLat;
    v.egoLonDeg = egoLon;

    v.lineCount = 0;
    for (int i = 0; i < segCount && v.lineCount < MapViewSnapshot::kMaxLines; i++) {
        MapLine &line = v.lines[v.lineCount];
        projectToScreen(segs[i].startLatE7 / 1e7f, segs[i].startLonE7 / 1e7f, egoLat, egoLon, projHeading,
                         scale, line.x1, line.y1);
        projectToScreen(segs[i].endLatE7 / 1e7f, segs[i].endLonE7 / 1e7f, egoLat, egoLon, projHeading, scale,
                         line.x2, line.y2);
        // Display class for line styling. The current DB writes roadClass=0
        // for every segment, so deriving it from the (populated) speed limit is
        // what actually gives the map varied, visible road styling — highways
        // bright/thick, small streets dim/thin. The current road still wins
        // (cyan highlight). If roadClass ever gets populated, it's respected.
        uint8_t dc;
        if (road.roadId != 0 && segs[i].id == road.roadId) {
            dc = kMapLineCurrentRoad;
        } else if (segs[i].roadClass != 0) {
            dc = segs[i].roadClass;
        } else {
            int16_t sl = segs[i].speedLimitKmh;
            dc = (sl >= 80) ? 1 : (sl >= 60) ? 2 : 3;
        }
        line.roadClass = dc;
        v.lineCount++;
    }

    // Trail breadcrumbs from SnapToRoad engine
    const TrailPoint *trailPts = nullptr;
    int trailCount = SnapToRoad::instance().getTrail(&trailPts);
    v.trailCount = 0;
    for (int i = 0; i < trailCount && v.trailCount < MapViewSnapshot::kMaxTrailPoints; i++) {
        projectToScreen(trailPts[i].latDeg, trailPts[i].lonDeg, egoLat, egoLon, projHeading, scale,
                         v.trailX[v.trailCount], v.trailY[v.trailCount]);
        v.trailCount++;
    }

    // Markers (cameras and signs)
    v.markerCount = 0;
    {
        NearbyMarkerRaw rawMarkers[MapViewSnapshot::kMaxMarkers];
        int markerCount = 0;
        speedLimitManagerGetNearbyMarkers(egoLat, egoLon, radiusM, rawMarkers, MapViewSnapshot::kMaxMarkers,
                                           &markerCount);
        for (int i = 0; i < markerCount; i++) {
            MapMarker &m = v.markers[v.markerCount];
            projectToScreen(rawMarkers[i].lat, rawMarkers[i].lon, egoLat, egoLon, projHeading, scale, m.x, m.y);
            m.kind = translateSignTypeToMarkerKind(rawMarkers[i].kind);
            v.markerCount++;
        }
    }

    v.generation = ++gGeneration;
    mapViewPublish(v);
}

// Throttle gate: recalculate only on movement >= 3m or turn >= 3 deg.
// kHeadingMinSpeedKmh (2026-09-24 V2): below this speed u-blox heading is pure
// noise — at a standstill it swings randomly, which used to fire the turn gate
// every tick, forcing a full-screen map redraw + full 33ms QSPI flush (and a
// visibly spinning map) while the car sat still. Ignore heading below it so a
// stopped/crawling car keeps a static map; the partial flush then makes those
// frames nearly free. Position movement still updates the map normally.
#ifdef VIETHUD_P4
// Finer steps on the 800x480 panel (full-frame render ~55 ms + map ~35 ms
// fits the 150 ms UI tick; the S3's 33 ms QSPI flush was why it stayed coarse).
static const float kMinMoveM = 1.5f;
static const float kMinTurnDeg = 1.5f;
#else
static const float kMinMoveM = 3.0f;
static const float kMinTurnDeg = 3.0f;
#endif
static const float kHeadingMinSpeedKmh = 8.0f;
// Near a standstill, GPS position also wanders (multipath) by several metres,
// which kept re-firing the move gate and redrawing the whole map (full 33ms
// flush) while the car was parked. Below kStopSpeedKmh require a much larger
// jump before redrawing, so a stopped car holds a static map. A genuine
// reposition (tunnel exit, GPS re-lock) still exceeds kStopMoveM and updates.
static const float kStopSpeedKmh = 3.0f;
static const float kStopMoveM = 12.0f;

static bool gHaveLast = false, gHaveLastHeading = false;
static float gLastLat = 0, gLastLon = 0, gLastHeading = 0;
static bool gNoFixHandled = false; // latch: no-fix state already drawn once (see mapRendererUpdate)

void mapRendererUpdate(const GnssSnapshot &gnss) {
    if (!gnss.fix) {
        // Recompute/republish only ONCE when the fix is first lost, not every
        // tick (2026-09-24 V2). The old code bumped the map generation on every
        // no-fix call, so with no GPS the whole map redrew + full-flushed every
        // frame (wasted power on the desk, and thrash if GPS drops mid-drive).
        // A regained fix falls through to the normal moved-gate below.
        if (!gNoFixHandled) {
            gNoFixHandled = true;
            mapRendererComputeFromSegments(gnss, nullptr, 0);
        }
        return;
    }
    gNoFixHandled = false; // valid fix: re-arm so the next loss redraws once more

    bool moved = !gHaveLast;
    if (!moved) {
        float dx = (gnss.lonDeg - gLastLon) * 111320.0f * cosf(gLastLat * (float)M_PI / 180.0f);
        float dy = (gnss.latDeg - gLastLat) * 110540.0f;
        float distM = sqrtf(dx * dx + dy * dy);
        float headErr = 0;
        if (gnss.headingValid && gHaveLastHeading && gnss.egoSpeedKmh >= kHeadingMinSpeedKmh) {
            float d = fmodf(fabsf(gnss.headingDeg - gLastHeading), 360.0f);
            headErr = d > 180.0f ? 360.0f - d : d;
        }
        float moveThresh = (gnss.egoSpeedKmh < kStopSpeedKmh) ? kStopMoveM : kMinMoveM;
        moved = distM >= moveThresh || headErr >= kMinTurnDeg;
    }
    if (!moved) return;

    gLastLat = gnss.latDeg;
    gLastLon = gnss.lonDeg;
    gHaveLast = true;
    if (gnss.headingValid) {
        gLastHeading = gnss.headingDeg;
        gHaveLastHeading = true;
    }

    float radiusM = mapRendererZoomRadiusM(gnss.egoSpeedKmh);
    static RoadSegment buf[MapViewSnapshot::kMaxLines];
    int count = 0;
    speedLimitManagerGetNearbySegments(gnss.latDeg, gnss.lonDeg, radiusM, buf, MapViewSnapshot::kMaxLines, &count);
    mapRendererComputeFromSegments(gnss, buf, count);
}

void mapRendererReset() {
    gHaveLast = false;
    gHaveLastHeading = false;
    gNoFixHandled = false;
    MapViewSnapshot blank;
    mapViewPublish(blank);
}

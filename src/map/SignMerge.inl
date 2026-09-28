// Implementation of mergeOsmLights() — see SignMerge.h.
#include <math.h>
#include <stdlib.h>

static inline int signMergeCmpLat(const void *a, const void *b) {
    int32_t x = *(const int32_t *)a, y = *(const int32_t *)b;
    return (x > y) - (x < y);
}

template <class SignT>
int mergeOsmLights(SignT *signs, int count, int cap, const OsmSignalPoint *osm, int nOsm, uint8_t lightType,
                   float dedupM, int32_t *scratch, int *outAdded, int *outDupes) {
    // Existing lights as (lat, index) pairs sorted by latitude.
    int nL = 0;
    for (int i = 0; i < count; i++) {
        if (signs[i].signType != lightType) continue;
        scratch[2 * nL] = signs[i].latE7;
        scratch[2 * nL + 1] = i;
        nL++;
    }
    qsort(scratch, nL, 2 * sizeof(int32_t), signMergeCmpLat);
    const int32_t win = (int32_t)(dedupM / 110540.0f * 1e7f) + 1;
    int added = 0, dupes = 0;
    for (int j = 0; j < nOsm && count < cap; j++) {
        const OsmSignalPoint &o = osm[j];
        int lo = 0, hi = nL;
        while (lo < hi) {
            int m = (lo + hi) >> 1;
            if (scratch[2 * m] < o.latE7 - win) lo = m + 1;
            else hi = m;
        }
        float kx = 111320.0f * cosf(o.latE7 * 1e-7f * 0.0174533f);
        bool dup = false;
        for (int m = lo; m < nL && scratch[2 * m] <= o.latE7 + win && !dup; m++) {
            const SignT &e = signs[scratch[2 * m + 1]];
            float dy = (e.latE7 - o.latE7) * 1e-7f * 110540.0f;
            float dx = (e.lonE7 - o.lonE7) * 1e-7f * kx;
            if (dx * dx + dy * dy >= dedupM * dedupM) continue;
            // Direction-aware (2026-09-28): a nearby card light only makes the OSM
            // light redundant if it already covers the same approach — it has no
            // direction (warns for every approach), or both are directed within
            // 60 deg. Otherwise the OSM light is kept: at a junction the card often
            // has a light for one approach only, and dropping the OSM lights of the
            // other approaches silently removed their warnings (192 in inner Hanoi).
            if (e.directionDeg == 0xFFFF) { dup = true; break; }
            if (o.directionDeg == 0xFFFF) continue;
            int d = (int)e.directionDeg - (int)o.directionDeg;
            d = ((d % 360) + 540) % 360 - 180;
            if (d < 0) d = -d;
            dup = d <= 60;
        }
        if (dup) {
            dupes++;
            continue;
        }
        SignT &p = signs[count++];
        p.latE7 = o.latE7;
        p.lonE7 = o.lonE7;
        p.directionDeg = o.directionDeg;
        p.signType = lightType;
        p.speedLimitKmh = 0;
        p.subType = 0;
        added++;
    }
    if (outAdded) *outAdded = added;
    if (outDupes) *outDupes = dupes;
    return count;
}

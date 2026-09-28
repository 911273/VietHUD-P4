// Host test for mergeOsmLights() (src/map/SignMerge.h) — the boot-time merge of
// the built-in OSM traffic lights into the card's sign list.
#include <cstdio>
#include <cstring>
#include <vector>
#include "SignMerge.h"

struct Sign { // same fields/names as SdCardManager.h's SignPt
    int32_t latE7, lonE7;
    uint16_t directionDeg;
    uint8_t signType, speedLimitKmh, subType;
};
const int kOsmTrafficSignalCount = 0;          // the real table isn't linked here
const OsmSignalPoint kOsmTrafficSignals[] = {{0, 0, 0}};

static int fails = 0, passes = 0;
#define CHECK(c, msg) do { if (c) { passes++; } else { fails++; printf("  FAIL: %s\n", msg); } } while (0)

static int32_t e7(double d) { return (int32_t)(d * 1e7 + (d >= 0 ? 0.5 : -0.5)); }
static double mN(double m) { return m / 110540.0; }                 // metres north -> deg
static double mE(double m, double lat) { return m / (111320.0 * cos(lat * 3.14159265 / 180.0)); }

int main() {
    const double lat = 21.03, lon = 105.85;
    const uint8_t LIGHT = 6, SPEED = 1;
    std::vector<Sign> s(64);
    int n = 0;
    s[n++] = {e7(lat), e7(lon), 90, LIGHT, 0, 0};                        // card light at origin
    s[n++] = {e7(lat + mN(200)), e7(lon), 0xFFFF, SPEED, 50, 0};         // unrelated speed sign
    s[n++] = {e7(lat + mN(500)), e7(lon + mE(10, lat)), 0xFFFF, LIGHT, 0, 0};
    OsmSignalPoint osm[] = {
        {e7(lat + mN(20)), e7(lon), 180},                 // 20 m, card light is 90 -> other approach -> kept
        {e7(lat + mN(15)), e7(lon), 120},                 // 15 m, 120 vs 90 (<=60) -> same approach -> duplicate
        {e7(lat), e7(lon + mE(29, lat)), 0xFFFF},         // 29 m, undirected OSM next to a directed card light -> kept
        {e7(lat), e7(lon + mE(31, lat)), 0xFFFF},         // 31 m -> kept
        {e7(lat + mN(200)), e7(lon), 0xFFFF},             // on the SPEED sign (not a light) -> kept
        {e7(lat + mN(505)), e7(lon + mE(10, lat)), 45},   // 5 m from an UNDIRECTED card light -> duplicate
        {e7(lat - mN(1000)), e7(lon), 270},               // far away -> kept, direction kept
    };
    const int nOsm = 7;
    int32_t scratch[2 * 64];
    int added = -1, dupes = -1;
    int n2 = mergeOsmLights(s.data(), n, (int)s.size(), osm, nOsm, LIGHT, 30.0f, scratch, &added, &dupes);
    printf("[1] dedupe against existing lights (30 m, direction-aware)\n");
    CHECK(added == 5 && dupes == 2, "5 added, 2 duplicates");
    CHECK(n2 == n + 5, "count grows by the added lights");
    bool far = false, onSpeed = false, east31 = false, otherApproach = false, sameApproach = false, undirNext = false;
    for (int i = n; i < n2; i++) {
        CHECK(s[i].signType == LIGHT && s[i].speedLimitKmh == 0 && s[i].subType == 0, "appended as a plain traffic light");
        if (s[i].latE7 == e7(lat - mN(1000))) far = (s[i].directionDeg == 270);
        if (s[i].latE7 == e7(lat + mN(200))) onSpeed = true;
        if (s[i].lonE7 == e7(lon + mE(31, lat))) east31 = (s[i].directionDeg == 0xFFFF);
        if (s[i].latE7 == e7(lat + mN(20)) && s[i].directionDeg == 180) otherApproach = true;
        if (s[i].latE7 == e7(lat + mN(15))) sameApproach = true;
        if (s[i].lonE7 == e7(lon + mE(29, lat))) undirNext = true;
    }
    CHECK(far, "OSM direction carried over");
    CHECK(onSpeed, "a non-light sign nearby does not suppress an OSM light");
    CHECK(east31, "31 m away is not a duplicate; unknown direction stays 0xFFFF");
    CHECK(otherApproach, "OSM light for another approach (90 deg off) is kept");
    CHECK(!sameApproach, "OSM light for the same approach (30 deg off) is a duplicate");
    CHECK(undirNext, "undirected OSM light next to a directed card light is kept");

    printf("[2] capacity is respected\n");
    std::vector<Sign> t(3);
    t[0] = {e7(lat), e7(lon), 0xFFFF, SPEED, 50, 0};
    int n3 = mergeOsmLights(t.data(), 1, 3, osm, nOsm, LIGHT, 30.0f, scratch, &added, &dupes);
    CHECK(n3 == 3 && added == 2, "stops at cap");

    printf("[3] empty card (no signs.bin)\n");
    std::vector<Sign> u(8);
    int n4 = mergeOsmLights(u.data(), 0, 8, osm, nOsm, LIGHT, 30.0f, scratch, &added, &dupes);
    CHECK(n4 == 7 && added == 7 && dupes == 0, "all OSM lights added");

    printf("\n==== %d passed, %d failed ====\n", passes, fails);
    return fails ? 1 : 0;
}

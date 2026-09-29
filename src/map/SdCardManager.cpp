#include "SdCardManager.h"
#include "OsmTrafficSignals.h" // built-in OSM traffic lights merged into the sign list
#include "SignMerge.h"         // mergeOsmLights() (host-tested)
#include "pincfg.h"
#include <Arduino.h>
#include <SD_MMC.h>
#ifdef VIETHUD_P4
#include <FFat.h>
#endif

// The filesystem every read/write below goes through. Always the microSD card
// on the S3; on the P4 board it falls back to the FAT partition in its 16MB
// flash ("ffat", holding the core speedmap data) when no card is present —
// see ensureSdMmcBegun(). Paths are FS-relative ("/speedmap/..."), so both
// work unchanged.
static fs::FS *gDataFs = &SD_MMC;
#define DATA_FS (*gDataFs)
static bool gDataOnFlash = false;
fs::FS &sdMgrDataFs() { return *gDataFs; }
bool sdMgrDataOnFlash() { return gDataOnFlash; }
static uint64_t dataFsTotalBytes() {
#ifdef VIETHUD_P4
    if (gDataOnFlash) return FFat.totalBytes();
#endif
    return SD_MMC.totalBytes();
}
static uint64_t dataFsUsedBytes() {
#ifdef VIETHUD_P4
    if (gDataOnFlash) return FFat.usedBytes();
#endif
    return SD_MMC.usedBytes();
}
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#ifdef VIETHUD_P4
#include <esp_ldo_regulator.h>
#endif // esp_task_wdt_reset() — see ensureSdMmcBegun()'s own comment
#include <mbedtls/sha256.h> // sdMgrSha256File() — data-install readback verify
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdio.h> // sscanf() — trip-log filename parsing in sdMgrListTripLogs()
#include <string.h>

// SD_MMC (the ESP32-S3's dedicated SDMMC hardware peripheral), NOT SD.h +
// SPIClass — see pincfg.h's SD_MMC_CLK_PIN/CMD_PIN/D0_PIN comment for the
// full story: this board wires its TF slot to 1-bit SD_MMC (3 signals,
// no CS pin exists in this mode), confirmed from the vendor-adjacent demo
// project's own working code (refob/Arduino_JC3248W535_LVGL9.4). An earlier
// SPI-based guess (wrong protocol, not just wrong pins) caused two distinct
// real-hardware regressions by forcing extra traffic onto the SPI2/SPI3
// peripherals the display and touch driver already depend on — SD_MMC's
// dedicated peripheral shares neither, which is what actually fixes it.

// Standard reflected CRC-32 (polynomial 0xEDB88320) — the same algorithm
// Python's zlib.crc32 uses, so build_speedmap.py's `zlib.crc32(index_bytes)`
// and this function agree on the same input bytes without either side
// needing the other's library. Hand-rolled rather than an ESP-IDF ROM CRC
// API: this project doesn't otherwise depend on ROM-level APIs, and a
// ~20-line table-based implementation is cheap enough not to need one.
// Incremental form: start with crc = 0xFFFFFFFF, feed chunks, final ^ 0xFFFFFFFF.
static uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t len) {
    static uint32_t table[256];
    static bool tableInit = false;
    if (!tableInit) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        tableInit = true;
    }
    for (size_t i = 0; i < len; i++) crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}

// Serializes every function below against the other (log/TripLogger.cpp's
// task calling sdMgrAppendLine() while map/SpeedLimitManager.cpp's task
// calls sdMgrReadTile(), say) — added 2026-09-16 once TripLogger became a
// second real caller task; SD_MMC is one physical peripheral, not reentrant
// across FreeRTOS tasks without a lock. RAII so every early `return false`
// already scattered through this file (unchanged) still releases it.
static SemaphoreHandle_t sdMutex = NULL;
struct SdLock {
    SdLock() {
        if (!sdMutex) sdMutex = xSemaphoreCreateMutex();
        xSemaphoreTake(sdMutex, portMAX_DELAY);
    }
    ~SdLock() { xSemaphoreGive(sdMutex); }
};

static bool mounted = false;
static SpeedMapMetadata metadata;

// Loaded once at mount() time and kept in RAM for the whole session.
// Allocated EXACTLY metadata.tileCount entries in PSRAM (not a fixed cap in
// static internal RAM) — sized 2026-09-16 for a real Northern-Vietnam
// extract (tools/map_builder/, classified roads only: motorway..tertiary)
// which produced 62081 tiles, dwarfing the earlier Hanoi-core test's 165.
// At 28 bytes/entry that's ~1.7MB, which would have been impossible in this
// board's 320KB internal RAM (the previous fixed 512-entry static array's
// whole reason for existing) but is a rounding error against 8MB PSRAM —
// same "move it to PSRAM, not internal RAM" reasoning
// SpeedLimitManager.cpp's tile segment CACHE already uses. No arbitrary cap
// to bump again next time the region grows: this just allocates what
// metadata.tileCount says, so the only real ceiling is PSRAM capacity
// itself (~285000 tiles at 8MB, far beyond any realistic single-country
// extract).
// In-RAM tile index (2026-09-26): only what the lookup + tile read use. The
// file's 28-byte TileIndexEntry also carries a per-tile bbox nothing reads; for
// the nationwide card (175k tiles) that was 4.9 MB of PSRAM — the single
// biggest reason PSRAM ran out (map canvas alloc failed -> boot loop).
struct TileIdx {
    uint32_t tileId;
    uint32_t fileOffset;
    uint32_t fileSize;
};
static TileIdx *tileIndex = NULL;
static int tileIndexCount = 0;

// Loaded once, independently of sdMgrMount()'s own tile-index loading (see
// this array's own header comment for why) — a flat CameraPoint[], small
// enough (real speed-camera counts are low thousands at most, 16
// bytes/entry) that PSRAM vs. internal RAM doesn't matter the way it does
// for tileIndex above; PSRAM chosen anyway for consistency with everything
// else this module keeps resident.
static CamPt *cameraPoints = NULL;
static int cameraPointCount = 0;
static SignPt *trafficSigns = NULL;
static int trafficSignCount = 0;

// PSRAM that must stay free after the camera/sign arrays are loaded, for the
// map canvas, LVGL heap growth, WiFi/TLS buffers and the tile cache.
static const size_t kPsramReserveBytes = 1536 * 1024;
static size_t psramRoomFor(size_t want) {
    size_t freeB = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t big = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    size_t room = freeB > kPsramReserveBytes ? freeB - kPsramReserveBytes : 0;
    if (room > big) room = big;
    return want < room ? want : room;
}
static int cmpCamLat(const void *a, const void *b) {
    int32_t x = ((const CamPt *)a)->latE7, y = ((const CamPt *)b)->latE7;
    return (x > y) - (x < y);
}
static int cmpSignLat(const void *a, const void *b) {
    int32_t x = ((const SignPt *)a)->latE7, y = ((const SignPt *)b)->latE7;
    return (x > y) - (x < y);
}
// Built-in OpenStreetMap traffic lights (src/map/OsmTrafficSignals.cpp,
// 2026-09-27): the WYN alert data has NO traffic lights in inner Hanoi (0 vs
// 364 signalized intersections in OSM). Appended as SIGN_TYPE_TRAFFIC_LIGHT
// (with the OSM traffic_signals:direction when tagged, else any approach),
// skipping any the card already has within 30 m. trafficSigns is unsorted here.
static void mergeOsmTrafficSignals() {
    const int nOsm = kOsmTrafficSignalCount;
    if (nOsm <= 0) return;
    size_t want = (size_t)(trafficSignCount + nOsm) * sizeof(SignPt);
    if (psramRoomFor(want) < want) {
        Serial.println("[sdmgr] WARN: no PSRAM room for built-in OSM traffic lights");
        return;
    }
    SignPt *grown = (SignPt *)heap_caps_realloc(trafficSigns, want, MALLOC_CAP_SPIRAM);
    if (!grown) return;
    trafficSigns = grown;
    int32_t *scratch = (int32_t *)heap_caps_malloc((size_t)(trafficSignCount + 1) * 2 * sizeof(int32_t), MALLOC_CAP_SPIRAM);
    if (!scratch) return;
    int added = 0, dupes = 0;
    trafficSignCount = mergeOsmLights(trafficSigns, trafficSignCount, trafficSignCount + nOsm, kOsmTrafficSignals, nOsm,
                                      (uint8_t)SIGN_TYPE_TRAFFIC_LIGHT, 30.0f, scratch, &added, &dupes);
    heap_caps_free(scratch);
    Serial.printf("[sdmgr] built-in OSM traffic lights: %d added, %d already on the card\n", added, dupes);
}

// Road Names Database in PSRAM. roadNameCount is uint32 (was uint16): the full-VN
// dataset can exceed 65 535 unique street names (nationwide is already ~52 175),
// so names.bin format v2 carries a 32-bit count + 32-bit per-segment name ids.
// segNameWidth records how wide each seg_names.bin entry is (2 for legacy v1
// cards, 4 for v2), so an old card still reads correctly. See the names.bin
// loader for the exact v1/v2 header layouts.
//
// NOT loaded into PSRAM any more (2026-09-26): the nationwide names.bin is a
// ~1.7 MB pool + ~0.25 MB offsets, which filled PSRAM so completely that
// allocations spilled into internal RAM and the WiFi driver could no longer
// start (esp_wifi_init ESP_ERR_NO_MEM, then a crash in hostap attach). The only
// consumer is the current-road label, which changes a few times a minute and
// is cached per segment, so each name is read straight from the card instead.
static uint32_t roadNameCount = 0;
static uint32_t roadNameOffsetsBase = 0; // file offset of the u32 offsets table
static uint32_t roadNamePoolBase = 0;    // file offset of the UTF-8 string pool
static size_t roadNamePoolSize = 0;
static uint8_t segNameWidth = 2; // bytes per seg_names.bin entry: v1=2, v2=4

bool sdMgrIsAvailable() { return mounted; }

// Brings up the underlying SD_MMC peripheral exactly once, however many of
// sdMgrMount()/sdMgrAppendLine() end up calling it first — added 2026-09-16
// when log/TripLogger.cpp's own task became a second caller that shouldn't
// have to wait on (or race) map/SpeedLimitManager.cpp's task calling
// sdMgrMount() first just to get a working card. Distinct from `mounted`
// above: that flag additionally requires a valid speedmap database
// (metadata.bin etc., see sdMgrMount() below), whereas this only means "the
// SD_MMC peripheral itself is up," which sdMgrAppendLine() alone needs.
static bool sdMmcBegun = false;
static uint32_t lastFailedAttemptMs = 0;
// Confirmed on real hardware 2026-09-16 (no SD card inserted): without this,
// log/TripLogger.cpp's task retries sdMgrAppendLine() every 200ms forever
// while its header write keeps failing, and EVERY one of those calls re-ran
// this whole function's ~400ms of blocking delay()s plus a full
// SD_MMC.begin() attempt — a tight, permanent loop hammering the peripheral
// and spamming the log roughly every 1.6s with no card ever going to appear.
// 30s is generous enough that a card inserted after boot is still picked up
// reasonably promptly, while cutting the retry rate by >100x when none is
// present at all.
static const uint32_t kRetryCooldownMs = 30000;

static bool ensureSdMmcBegun() {
    if (sdMmcBegun) return true;
    uint32_t now = millis();
    // lastFailedAttemptMs starts at 0, which would look identical to "an
    // attempt just failed at time 0" — but millis() reads >0 by the time any
    // task is far enough into its loop to call this, so a real first attempt
    // is never mistakenly skipped by this check.
    if (lastFailedAttemptMs != 0 && now - lastFailedAttemptMs < kRetryCooldownMs) return false;
    lastFailedAttemptMs = now; // set up front — every early return below is a failed attempt

#ifdef VIETHUD_P4
    // JC4880P443C: 4-bit SDMMC slot0 on IO_MUX pins, and TF_VCC is fed by the
    // P4's on-chip LDO channel 4 (always-on P-FET, GPIO45 enable not fitted) —
    // without it the card is unpowered and init times out with 0x107, which
    // is exactly what the factory fw logged. SD_MMC must not grab an LDO
    // itself (setPowerChannel(-1)) or it collides with this one.
    {
        static esp_ldo_channel_handle_t sSdLdo = nullptr;
        if (!sSdLdo) {
            esp_ldo_channel_config_t lc = {};
            lc.chan_id = SD_LDO_CHAN;
            lc.voltage_mv = 3300;
            if (esp_ldo_acquire_channel(&lc, &sSdLdo) != ESP_OK) Serial.println("[sdmgr] LDO4 (TF_VCC) acquire failed");
            delay(50);
        }
        SD_MMC.end();
        SD_MMC.setPins(SD_MMC_CLK_PIN, SD_MMC_CMD_PIN, SD_MMC_D0_PIN, SD_MMC_D1_PIN, SD_MMC_D2_PIN, SD_MMC_D3_PIN);
        SD_MMC.setPowerChannel(-1);
        static const int kFreqs[] = {SDMMC_FREQ_HIGHSPEED, SDMMC_FREQ_DEFAULT, SDMMC_FREQ_PROBING};
        for (int f : kFreqs) {
            if (esp_task_wdt_status(NULL) == ESP_OK) esp_task_wdt_reset(); // caller task may not be subscribed (IDF 5 logs an error then)
            if (SD_MMC.begin("/sdmmc", false, false, f)) {
                sdMmcBegun = true;
                Serial.printf("[sdmgr] SD mounted (4-bit, %d kHz): %llu MB, type %d\n", f,
                              SD_MMC.cardSize() / (1024 * 1024), SD_MMC.cardType());
                return true;
            }
            SD_MMC.end();
            delay(50);
        }
        Serial.println("[sdmgr] SD_MMC.begin() failed — no card detected");
        // No card: use the core data flashed into the "ffat" partition
        // (partitions_p4_data.csv; image built by `pio run -t buildfs`).
        if (FFat.begin(false, "/ffat", 10, "ffat") && FFat.exists("/speedmap/metadata.bin")) {
            gDataFs = &FFat;
            gDataOnFlash = true;
            sdMmcBegun = true;
            Serial.printf("[sdmgr] using speedmap data from internal flash (FFat %u/%u KB)\n",
                          (unsigned)(FFat.usedBytes() / 1024), (unsigned)(FFat.totalBytes() / 1024));
            return true;
        }
        Serial.println("[sdmgr] no flash data partition either (flash it with -t uploadfs)");
        return false;
    }
#endif

    // Tear the peripheral down before priming/re-initialising it (added
    // 2026-09-21). Confirmed on real hardware: after a run of rapid
    // upload-triggered soft resets, SD_MMC.begin() started failing with
    // sdmmc_init_ocr / send_op_cond error 0x107 ("no card detected") on
    // EVERY retry for a whole boot, and only a true power cycle brought it
    // back — the exact same class of "a soft reset resets the ESP32, NOT
    // the peripheral" failure this project already hit with the AXS15231B
    // touch controller. A soft reset can leave the SDMMC host half-
    // initialised from the previous run, and begin() on top of that can't
    // recover; end() first puts it back to a known state. Deliberately
    // BEFORE the clock-priming below, not after — end() reconfigures these
    // same pins, so priming first would just be undone. Harmless when
    // nothing was ever begun (end() on an un-begun host is a no-op).
    SD_MMC.end();
    delay(10);

    // Clock-priming sequence lifted verbatim from the vendor-adjacent demo
    // (mp3_player.ino) — sends the card a couple of clock transitions with
    // CMD held high (pulled up) before SD_MMC.begin() proper, a well-known
    // SD initialization quirk (cards can need "warm-up" clocks with
    // CS/CMD high to settle into the right mode). Kept even though this
    // project can't independently verify it's load-bearing on this exact
    // board, on the same "a hard-won hardware fix from someone who tested
    // it stays unless disproven" principle this project applies to its own
    // discoveries (e.g. the touch controller's 100kHz requirement).
    pinMode(SD_MMC_CMD_PIN, INPUT_PULLUP);
    pinMode(SD_MMC_D0_PIN, INPUT_PULLUP);
    pinMode(SD_MMC_CLK_PIN, OUTPUT);
    digitalWrite(SD_MMC_CLK_PIN, LOW);
    delay(200);
    digitalWrite(SD_MMC_CLK_PIN, HIGH);
    delay(200);

    if (!SD_MMC.setPins(SD_MMC_CLK_PIN, SD_MMC_CMD_PIN, SD_MMC_D0_PIN)) {
        Serial.println("[sdmgr] SD_MMC.setPins() failed");
        return false;
    }
    // esp_task_wdt_reset() calls threaded through this function's 3-stage
    // retry — added 2026-09-22 after a REAL, reproducible field crash:
    // "Task watchdog got triggered ... speedLimitTask ... Aborting." A
    // failing card makes each of the 3 SD_MMC.begin() attempts below block
    // for real time (its own internal command retries at the hardware
    // level, worse at the slower fallback frequencies), on top of the
    // explicit delay()s already here — confirmed on hardware to add up to
    // several seconds for one full failed call. speedLimitTaskFn's own
    // retry-mount logic calls sdMgrMount() (which calls this) directly
    // inline in its loop, before that loop's own esp_task_wdt_reset() — so
    // a single slow call here could burn through the whole per-task
    // watchdog budget before ever reaching it. A no-op (returns an error
    // code, nothing worse) if the calling task was never registered via
    // esp_task_wdt_add(), so this is safe from every caller of
    // ensureSdMmcBegun() (map/SpeedLimitManager.cpp, log/TripLogger.cpp,
    // etc.), not just the one that actually crashed.
    esp_task_wdt_reset();
    if (SD_MMC.begin("/sdmmc", true, false, SDMMC_FREQ_DEFAULT)) {
        sdMmcBegun = true;
        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        Serial.printf("[sdmgr] SD Card mounted successfully! Size: %llu MB, Type: %d\n", cardSize, SD_MMC.cardType());
        return true;
    }
    SD_MMC.end();
    delay(100);
    esp_task_wdt_reset();
    if (SD_MMC.begin("/sdmmc", true, false, 10000)) {
        sdMmcBegun = true;
        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        Serial.printf("[sdmgr] SD Card mounted at 10MHz fallback! Size: %llu MB\n", cardSize);
        return true;
    }
    SD_MMC.end();
    delay(100);
    esp_task_wdt_reset();
    if (SD_MMC.begin("/sdmmc", true, false, SDMMC_FREQ_PROBING)) {
        sdMmcBegun = true;
        uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
        Serial.printf("[sdmgr] SD Card mounted at 400kHz fallback! Size: %llu MB\n", cardSize);
        return true;
    }
    esp_task_wdt_reset();
    Serial.println("[sdmgr] SD_MMC.begin() failed — no card detected");
    return false;
}

bool sdMgrMount() {
    SdLock lock;
    if (mounted) return true;
    mounted = false;
    if (!ensureSdMmcBegun()) return false;

    // Paths are relative to the SD_MMC filesystem's own root, NOT prefixed
    // with the "/sdmmc" mountpoint string passed to begin() above — that
    // string only registers the VFS mount internally (confirmed against
    // the vendor demo's own usage: it does SD_MMC.begin("/sdmmc", ...) but
    // then DATA_FS.open("/music"), not DATA_FS.open("/sdmmc/music")).
    File metaFile = DATA_FS.open("/speedmap/metadata.bin");
    if (!metaFile) {
        Serial.println("[sdmgr] /speedmap/metadata.bin not found");
        return false;
    }
    size_t got = metaFile.read((uint8_t *)&metadata, sizeof(metadata));
    metaFile.close();
    if (got != sizeof(metadata)) {
        Serial.printf("[sdmgr] metadata.bin too short (%u of %u bytes) — MAP ERROR\n", (unsigned)got,
                      (unsigned)sizeof(metadata));
        return false;
    }
    if (memcmp(metadata.magic, SPEEDMAP_MAGIC, 4) != 0) {
        Serial.println("[sdmgr] metadata.bin magic mismatch — not a speedmap database, MAP ERROR");
        return false;
    }
    if (metadata.formatVersion != SPEEDMAP_FORMAT_VERSION) {
        Serial.printf("[sdmgr] metadata.bin format_version=%u, firmware supports %u — MAP FORMAT ERROR\n",
                      metadata.formatVersion, SPEEDMAP_FORMAT_VERSION);
        return false;
    }

    File idxFile = DATA_FS.open("/speedmap/index.bin");
    if (!idxFile) {
        Serial.println("[sdmgr] /speedmap/index.bin not found — MAP ERROR");
        return false;
    }
    size_t idxSize = idxFile.size();
    if (idxSize != metadata.tileCount * sizeof(TileIndexEntry)) {
        Serial.printf("[sdmgr] index.bin size %u doesn't match tileCount*%u — MAP ERROR\n", (unsigned)idxSize,
                      (unsigned)sizeof(TileIndexEntry));
        idxFile.close();
        return false;
    }
    // Free a previous mount's buffer before allocating a new one — sdMgrMount()
    // isn't currently called more than once per boot, but this keeps a
    // second call (e.g. a future "reload map" feature) from leaking PSRAM
    // instead of silently assuming it never happens.
    if (tileIndex) {
        heap_caps_free(tileIndex);
        tileIndex = NULL;
    }
    tileIndex = (TileIdx *)heap_caps_malloc((size_t)metadata.tileCount * sizeof(TileIdx), MALLOC_CAP_SPIRAM);
    TileIndexEntry *idxChunk = tileIndex ? (TileIndexEntry *)malloc(256 * sizeof(TileIndexEntry)) : NULL;
    tileIndexCount = (int)metadata.tileCount;
    if (!idxChunk) {
        if (tileIndex) { heap_caps_free(tileIndex); tileIndex = NULL; }
        Serial.printf("[sdmgr] PSRAM allocation for %u tile index entries skipped - using on-demand file index (0 KB PSRAM)\n",
                      (unsigned)metadata.tileCount);
        idxFile.close();
    } else {
        uint32_t crc = 0xFFFFFFFFu;
        int done = 0;
        while (done < tileIndexCount) {
            int want = tileIndexCount - done < 256 ? tileIndexCount - done : 256;
            size_t got = idxFile.read((uint8_t *)idxChunk, want * sizeof(TileIndexEntry));
            if (got != want * sizeof(TileIndexEntry)) break;
            crc = crc32Update(crc, (const uint8_t *)idxChunk, got);
            for (int i = 0; i < want; i++) {
                tileIndex[done + i].tileId = idxChunk[i].tileId;
                tileIndex[done + i].fileOffset = idxChunk[i].fileOffset;
                tileIndex[done + i].fileSize = idxChunk[i].fileSize;
            }
            done += want;
        }
        free(idxChunk);
        idxFile.close();
        uint32_t computedCrc = crc ^ 0xFFFFFFFFu;
        if (done != tileIndexCount) {
            Serial.println("[sdmgr] short read on index.bin - falling back to file index");
            heap_caps_free(tileIndex);
            tileIndex = NULL;
        } else if (computedCrc != metadata.crc32) {
            Serial.printf("[sdmgr] index.bin CRC mismatch (computed 0x%08lX, expected 0x%08lX) - falling back to file index\n",
                          (unsigned long)computedCrc, (unsigned long)metadata.crc32);
            heap_caps_free(tileIndex);
            tileIndex = NULL;
        } else {
            Serial.printf("[sdmgr] tile index in PSRAM: %u KB (compact)\n",
                          (unsigned)((size_t)tileIndexCount * sizeof(TileIdx) / 1024));
        }
    }

    Serial.printf("[sdmgr] mounted OK — region=%.16s version=%.16s tiles=%d\n", metadata.region, metadata.mapVersion,
                  tileIndexCount);
    mounted = true;

    // cameras.bin — loaded independently of the metadata/index validation
    // above, and never fails sdMgrMount() itself: this is an OPTIONAL
    // extra (see CameraPoint's own SpeedMapFormat.h comment), so a card
    // built before it existed, or a region export with genuinely zero
    // speed cameras, both correctly end up with cameraPointCount==0
    // rather than a MAP ERROR. Streamed in chunks into compact CamPt records
    // (see SdCardManager.h), never more than psramRoomFor() allows.
    if (cameraPoints) {
        heap_caps_free(cameraPoints);
        cameraPoints = NULL;
        cameraPointCount = 0;
    }
    static const int kChunk = 256;
    File camFile = DATA_FS.open("/speedmap/cameras.bin");
    if (camFile) {
        int n = (int)(camFile.size() / sizeof(CameraPoint));
        int cap = (int)(psramRoomFor((size_t)n * sizeof(CamPt)) / sizeof(CamPt));
        if (cap < n) Serial.printf("[sdmgr] WARN: PSRAM room for only %d of %d cameras\n", cap, n);
        if (cap > 0) cameraPoints = (CamPt *)heap_caps_malloc((size_t)cap * sizeof(CamPt), MALLOC_CAP_SPIRAM);
        CameraPoint *chunk = cameraPoints ? (CameraPoint *)malloc(kChunk * sizeof(CameraPoint)) : NULL;
        if (chunk) {
            // Field-order repair (2026-09-26, "bieu tuong canh bao toc do bi sai"):
            // tools/viethud_builder.py and merge_vietmap_papago.py packed cameras
            // as (heading, speed) instead of (speedLimitKmh, directionDeg), so the
            // camera card showed the camera's compass bearing (135, 178, ...) as
            // its limit. Detected on the file's first chunk: a real share of
            // impossible limits (>150) means the two fields are swapped.
            int swappedState = -1, cleared = 0;
            while (cameraPointCount < cap) {
                int want = cap - cameraPointCount < kChunk ? cap - cameraPointCount : kChunk;
                int got = (int)(camFile.read((uint8_t *)chunk, want * sizeof(CameraPoint)) / sizeof(CameraPoint));
                if (got <= 0) break;
                if (swappedState < 0) {
                    int impossible = 0;
                    for (int i = 0; i < got; i++)
                        if (chunk[i].speedLimitKmh > 150) impossible++;
                    swappedState = impossible * 20 > got ? 1 : 0;
                }
                for (int i = 0; i < got; i++) {
                    int sp = chunk[i].speedLimitKmh, hd = chunk[i].directionDeg;
                    if (swappedState == 1) {
                        int t = sp;
                        sp = hd;
                        hd = (t >= 0 && t < 360) ? t : 0xFFFF;
                    }
                    if (sp != -1 && (sp < 5 || sp > 150 || sp % 5 != 0)) {
                        sp = -1;
                        cleared++;
                    }
                    CamPt &o = cameraPoints[cameraPointCount++];
                    o.latE7 = chunk[i].latE7;
                    o.lonE7 = chunk[i].lonE7;
                    o.speedLimitKmh = (int16_t)sp;
                    o.directionDeg = (uint16_t)hd;
                }
            }
            free(chunk);
            qsort(cameraPoints, cameraPointCount, sizeof(CamPt), cmpCamLat);
            Serial.printf("[sdmgr] cameras.bin: %s, %d invalid limit(s) -> unknown\n",
                          swappedState == 1 ? "speed/direction fields were SWAPPED (old builder) — repaired"
                                            : "field order OK",
                          cleared);
        } else if (n > 0) {
            Serial.println("[sdmgr] PSRAM allocation for camera points failed — continuing with zero cameras");
        }
        camFile.close();
    }
    Serial.printf("[sdmgr] cameras.bin: %d speed camera(s) loaded\n", cameraPointCount);

    // signs.bin — compact SignPt records; camera rows (type 4) are skipped,
    // they duplicate cameras.bin. Allocated for the header count, then shrunk.
    if (trafficSigns) {
        heap_caps_free(trafficSigns);
        trafficSigns = NULL;
        trafficSignCount = 0;
    }
    File signFile = DATA_FS.open("/speedmap/signs.bin");
    if (signFile) {
        TrafficSignHeader hdr;
        if (signFile.read((uint8_t *)&hdr, sizeof(hdr)) == sizeof(hdr) &&
            memcmp(hdr.magic, SIGN_MAGIC, 4) == 0 && hdr.signCount > 0) {
            int n = (int)hdr.signCount;
            int cap = (int)(psramRoomFor((size_t)n * sizeof(SignPt)) / sizeof(SignPt));
            if (cap > 0) trafficSigns = (SignPt *)heap_caps_malloc((size_t)cap * sizeof(SignPt), MALLOC_CAP_SPIRAM);
            TrafficSignPoint *chunk = trafficSigns ? (TrafficSignPoint *)malloc(kChunk * sizeof(TrafficSignPoint)) : NULL;
            if (chunk) {
                int readTotal = 0, skippedCam = 0;
                while (readTotal < n && trafficSignCount < cap) {
                    int want = n - readTotal < kChunk ? n - readTotal : kChunk;
                    int got = (int)(signFile.read((uint8_t *)chunk, want * sizeof(TrafficSignPoint)) /
                                    sizeof(TrafficSignPoint));
                    if (got <= 0) break;
                    readTotal += got;
                    for (int i = 0; i < got && trafficSignCount < cap; i++) {
                        if (chunk[i].signType == SIGN_TYPE_CAMERA) { skippedCam++; continue; }
                        SignPt &o = trafficSigns[trafficSignCount++];
                        o.latE7 = chunk[i].latE7;
                        o.lonE7 = chunk[i].lonE7;
                        o.directionDeg = chunk[i].directionDeg;
                        o.signType = chunk[i].signType;
                        o.speedLimitKmh = chunk[i].speedLimitKmh;
                        o.subType = chunk[i].subType;
                    }
                }
                free(chunk);
                if (readTotal < n) Serial.printf("[sdmgr] WARN: PSRAM room for only part of signs.bin (%d of %d read)\n", readTotal, n);
                Serial.printf("[sdmgr] signs.bin: %d camera row(s) skipped (in cameras.bin)\n", skippedCam);
            } else {
                Serial.println("[sdmgr] PSRAM allocation for traffic signs failed");
            }
        }
        signFile.close();
    }
    Serial.printf("[sdmgr] signs.bin: %d traffic sign(s) loaded\n", trafficSignCount);
    mergeOsmTrafficSignals();
    if (trafficSigns && trafficSignCount > 0) {
        void *shrunk = heap_caps_realloc(trafficSigns, (size_t)trafficSignCount * sizeof(SignPt), MALLOC_CAP_SPIRAM);
        if (shrunk) trafficSigns = (SignPt *)shrunk;
        qsort(trafficSigns, trafficSignCount, sizeof(SignPt), cmpSignLat);
    }
    Serial.printf("[sdmgr] PSRAM after points: %u KB free, largest block %u KB\n",
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
                  (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024));

    // names.bin — header only; names are read on demand (sdMgrGetRoadName).
    roadNameCount = 0;
    roadNamePoolSize = 0;
    segNameWidth = 2;
    File nameFile = DATA_FS.open("/speedmap/names.bin");
    if (nameFile) {
        char magic[4];
        uint16_t version = 0;
        uint32_t count = 0, totalBytes = 0;
        if (nameFile.read((uint8_t *)magic, 4) == 4 && memcmp(magic, "VNNM", 4) == 0) {
            nameFile.read((uint8_t *)&version, 2);
            // Header layouts, both 16 bytes total including magic+version:
            //   v1: count(u16) totalBytes(u32) reserved(u32)   — seg_names entries are u16
            //   v2: count(u32) totalBytes(u32) reserved(u16)   — seg_names entries are u32
            // v2 lifts the 65 535-name ceiling for the full-VN dataset. Old v1
            // cards keep working unchanged.
            if (version >= 2) {
                uint16_t reserved16 = 0;
                nameFile.read((uint8_t *)&count, 4);
                nameFile.read((uint8_t *)&totalBytes, 4);
                nameFile.read((uint8_t *)&reserved16, 2);
                segNameWidth = 4;
            } else {
                uint16_t count16 = 0;
                uint32_t reserved32 = 0;
                nameFile.read((uint8_t *)&count16, 2);
                nameFile.read((uint8_t *)&totalBytes, 4);
                nameFile.read((uint8_t *)&reserved32, 4);
                count = count16;
                segNameWidth = 2;
            }
            // Sanity: the file must actually contain the table + pool it declares
            // (a truncated/corrupt names.bin simply disables street names).
            uint32_t offBase = 16, poolBase = 16 + count * 4;
            if (count > 0 && totalBytes > 0 && count < 5000000u && totalBytes < 16000000u &&
                nameFile.size() >= (size_t)poolBase + totalBytes) {
                roadNameCount = count;
                roadNameOffsetsBase = offBase;
                roadNamePoolBase = poolBase;
                roadNamePoolSize = totalBytes;
                Serial.printf("[sdmgr] names.bin v%u: %u street names (read on demand, %u KB on card, seg id width %uB)\n",
                              (unsigned)version, (unsigned)roadNameCount, (unsigned)(roadNamePoolSize / 1024),
                              (unsigned)segNameWidth);
            } else {
                Serial.println("[sdmgr] names.bin header/size mismatch — street names disabled");
            }
        }
        nameFile.close();
    }

    return true;
}

bool sdMgrGetMetadata(SpeedMapMetadata &out) {
    SdLock lock;
    if (!mounted) return false;
    out = metadata;
    return true;
}

bool sdMgrGetCameras(const CamPt **out, int *outCount) {
    SdLock lock;
    *out = cameraPoints;
    *outCount = cameraPointCount;
    return true; // 0 cameras is a normal, successful result — see this array's own comment
}

bool sdMgrGetSigns(const SignPt **out, int *outCount) {
    SdLock lock;
    *out = trafficSigns;
    *outCount = trafficSignCount;
    return true;
}



bool sdMgrReadBytes(const char *path, uint32_t offset, uint8_t *outBuf, size_t len) {
    SdLock lock;
    if (!ensureSdMmcBegun() || !path || !outBuf || len == 0) return false;
    File f = DATA_FS.open(path, FILE_READ);
    if (!f) return false;
    if (offset > 0 && !f.seek(offset)) {
        f.close();
        return false;
    }
    size_t got = f.read(outBuf, len);
    f.close();
    return got == len;
}

bool sdMgrReadTile(const TileIndexEntry &entry, RoadSegment *outBuf, int maxSegments, int *outCount) {
    SdLock lock;
    *outCount = 0;
    if (!mounted) return false;

    // One shared blob for every tile (format V2) — opened fresh per call
    // rather than kept as a persistent handle, same "no extra state to get
    // wrong" reasoning every other read in this file already follows; SD_MMC
    // open() is cheap compared to the seek+read that follows anyway.
    File f = DATA_FS.open("/speedmap/tiles.bin");
    if (!f) {
        Serial.println("[sdmgr] /speedmap/tiles.bin not found (treating tile as empty)");
        return false;
    }
    if (!f.seek(entry.fileOffset)) {
        Serial.printf("[sdmgr] seek to offset %lu in tiles.bin failed\n", (unsigned long)entry.fileOffset);
        f.close();
        return false;
    }
    int available = (int)(entry.fileSize / sizeof(RoadSegment));
    int toRead = available < maxSegments ? available : maxSegments;
    size_t got = f.read((uint8_t *)outBuf, toRead * sizeof(RoadSegment));
    f.close();
    *outCount = (int)(got / sizeof(RoadSegment));
    // Sanitize impossible limits from raw OSM maxspeed tags (found by the
    // 2026-09-29 all-limits sweep: 2, and 6060 from a "60;60"-style tag).
    // A doubled value decodes to itself; anything else outside 5..130 km/h
    // becomes UNKNOWN rather than being shown or spoken. Odd-but-plausible
    // values (51, 87, 89) are left as the data says.
    for (int i = 0; i < *outCount; i++) {
        int16_t &lim = outBuf[i].speedLimitKmh;
        if (lim < 0 || (lim >= 5 && lim <= 130)) continue;
        int16_t fixed = -1;
        if (lim >= 1000 && lim % 100 == lim / 100 && lim % 100 >= 5 && lim % 100 <= 130) fixed = lim % 100;
        static uint32_t sLogged = 0;
        if (sLogged++ < 8)
            Serial.printf("[sdmgr] seg %lu: invalid speed limit %d -> %d\n", (unsigned long)outBuf[i].id, lim, fixed);
        lim = fixed;
    }
    return true;
}

bool sdMgrDumpFileToSerial(const char *path) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    File f = DATA_FS.open(path, FILE_READ);
    if (!f) {
        Serial.printf("[sdmgr] dump: open failed: %s\n", path);
        return false;
    }
    Serial.printf("-----BEGIN FILE %s (%u bytes)-----\n", path, (unsigned)f.size());
    uint8_t buf[256];
    while (true) {
        size_t got = f.read(buf, sizeof(buf));
        if (got == 0) break;
        Serial.write(buf, got);
    }
    Serial.println();
    Serial.printf("-----END FILE %s-----\n", path);
    f.close();
    return true;
}

void sdMgrSummarizeTripLogs(uint32_t firstSessionId, uint32_t lastSessionId) {
    SdLock lock;
    if (!ensureSdMmcBegun()) {
        Serial.println("[sdmgr] summarize: SD_MMC not available");
        return;
    }
    for (uint32_t id = firstSessionId; id <= lastSessionId; id++) {
        char path[48];
        snprintf(path, sizeof(path), "/triplog/session_%04lu.csv", (unsigned long)id);
        File f = DATA_FS.open(path, FILE_READ);
        if (!f) continue; // this session number just never got a file (e.g. logging was off that boot) — not an error
        size_t sizeBytes = f.size();
        int lineCount = 0;
        float maxEgoKmh = 0;
        bool anySpeeding = false, anyEvent = false;
        f.readStringUntil('\n'); // skip header
        while (f.available()) {
            String line = f.readStringUntil('\n');
            if (line.length() == 0) continue;
            lineCount++;
            // tMs,egoKmh,limitValid,limitKmh,speeding,cameraAheadM,signType,signDistM,event
            // (log/TripLogger.cpp's schema — updated 2026-09-21 when radar
            // removal replaced its old activeTargets/primDistM/TTC columns
            // with speed-limit/camera/sign fields)
            int c1 = line.indexOf(',');
            int c2 = line.indexOf(',', c1 + 1);
            int c3 = line.indexOf(',', c2 + 1);
            int c4 = line.indexOf(',', c3 + 1);
            if (c1 < 0 || c2 < 0 || c3 < 0 || c4 < 0) continue;
            float egoKmh = line.substring(c1 + 1, c2).toFloat();
            int speeding = line.substring(c3 + 1, c4).toInt();
            if (egoKmh > maxEgoKmh) maxEgoKmh = egoKmh;
            if (speeding > 0) anySpeeding = true;
            if (line.endsWith("EVENT")) anyEvent = true;
        }
        f.close();
        Serial.printf("[sdmgr] session_%04lu: %uB lines=%d maxEgoKmh=%.1f anySpeeding=%d anyEvent=%d\n",
                      (unsigned long)id, (unsigned)sizeBytes, lineCount, (double)maxEgoKmh, (int)anySpeeding,
                      (int)anyEvent);
    }
}

int sdMgrListTripLogs(uint32_t *outIds, uint32_t *outSizes, int maxCount) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return 0;
    File dir = DATA_FS.open("/triplog");
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return 0;
    }
    int n = 0;
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        // name() is the bare filename on this core's SD_MMC (no directory
        // prefix); parse "session_NNNN.csv" and skip anything else that
        // happens to be in the directory.
        const char *name = f.name();
        const char *lastSlash = strrchr(name, '/');
        if (lastSlash) name = lastSlash + 1;
        unsigned id = 0;
        if (!f.isDirectory() && sscanf(name, "session_%u.csv", &id) == 1) {
            if (n < maxCount) {
                outIds[n] = (uint32_t)id;
                outSizes[n] = (uint32_t)f.size();
                n++;
            } else {
                // Keep the NEWEST maxCount, not the first maxCount the
                // directory happens to hand back (added 2026-09-21 after a
                // real card with 181 sessions on it returned ids 55-118 —
                // i.e. only old bench-test boots, with the drive anyone
                // actually wants never listed at all). Session ids only
                // ever increase, so "newest" is just "largest id": once
                // full, evict the smallest whenever a larger one shows up.
                int smallest = 0;
                for (int i = 1; i < n; i++)
                    if (outIds[i] < outIds[smallest]) smallest = i;
                if ((uint32_t)id > outIds[smallest]) {
                    outIds[smallest] = (uint32_t)id;
                    outSizes[smallest] = (uint32_t)f.size();
                }
            }
        }
        f.close();
    }
    dir.close();
    return n;
}

int sdMgrDeleteAllTripLogs() {
    SdLock lock;
    if (!ensureSdMmcBegun()) return -1;
    if (gDataOnFlash) return -1; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    File dir = DATA_FS.open("/triplog");
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return 0;
    }
    // Collect session IDs first (4 bytes each, not full path strings — keeps
    // internal RAM tiny), THEN remove: deleting while an openNextFile()
    // iteration is live is unreliable on SD_MMC.
    static uint32_t ids[128];
    int n = 0;
    for (File f = dir.openNextFile(); f && n < 128; f = dir.openNextFile()) {
        const char *name = f.name();
        const char *lastSlash = strrchr(name, '/');
        const char *bare = lastSlash ? lastSlash + 1 : name;
        unsigned id = 0;
        if (!f.isDirectory() && sscanf(bare, "session_%u.csv", &id) == 1) ids[n++] = (uint32_t)id;
        f.close();
    }
    dir.close();
    int deleted = 0;
    char path[40];
    for (int i = 0; i < n; i++) {
        snprintf(path, sizeof(path), "/triplog/session_%04lu.csv", (unsigned long)ids[i]);
        if (DATA_FS.remove(path)) deleted++;
    }
    Serial.printf("[sdmgr] deleted %d/%d trip log(s)\n", deleted, n);
    return deleted;
}

int sdMgrReadFileChunk(const char *path, size_t offset, uint8_t *buf, size_t bufSize) {
    SdLock lock;
    if (!ensureSdMmcBegun() || !DATA_FS.exists(path)) return -1; // quiet "not found" for optional files
    File f = DATA_FS.open(path, FILE_READ);
    if (!f) return -1;
    if (offset >= f.size()) {
        f.close();
        return 0;
    }
    if (!f.seek(offset)) {
        f.close();
        return -1;
    }
    int got = (int)f.read(buf, bufSize);
    f.close();
    return got;
}

bool sdMgrAppendLine(const char *path, const char *line) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    if (gDataOnFlash) return false; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)

    // FILE_APPEND creates the file if missing, but NOT missing parent
    // directories — DATA_FS.mkdir() one level up first if the target isn't
    // at the filesystem root. Single-level only (this project's only caller,
    // log/TripLogger.cpp, uses one fixed "/triplog" directory) — not a
    // recursive mkdir -p, since there's no current need for nested paths.
    const char *slash = strrchr(path, '/');
    if (slash && slash != path) {
        char dir[64];
        size_t dirLen = (size_t)(slash - path);
        if (dirLen < sizeof(dir)) {
            memcpy(dir, path, dirLen);
            dir[dirLen] = '\0';
            if (!DATA_FS.exists(dir)) DATA_FS.mkdir(dir);
        }
    }

    File f = DATA_FS.open(path, FILE_APPEND);
    if (!f) {
        Serial.printf("[sdmgr] append open failed: %s\n", path);
        return false;
    }
    f.println(line);
    f.close();
    return true;
}

// Append raw bytes to a file (creates it + one-level parent dir if missing),
// mutex-guarded like the rest of this module. Open-append-close per call so the
// SD mutex is only held for one chunk at a time — the map-matcher task can still
// read tiles between chunks during a long online data download (net/
// DataUpdater.cpp). Returns true only if the full len was written.
bool sdMgrAppendBytes(const char *path, const uint8_t *buf, size_t len) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    if (gDataOnFlash) return false; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    const char *slash = strrchr(path, '/');
    if (slash && slash != path) {
        char dir[64];
        size_t dirLen = (size_t)(slash - path);
        if (dirLen < sizeof(dir)) {
            memcpy(dir, path, dirLen);
            dir[dirLen] = '\0';
            if (!DATA_FS.exists(dir)) DATA_FS.mkdir(dir);
        }
    }
    File f = DATA_FS.open(path, FILE_APPEND);
    if (!f) return false;
    size_t wrote = f.write(buf, len);
    f.close();
    return wrote == len;
}

bool sdMgrRemove(const char *path) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    if (gDataOnFlash) return false; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    if (!DATA_FS.exists(path)) return true; // already gone = success
    return DATA_FS.remove(path);
}

bool sdMgrRename(const char *from, const char *to) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    if (gDataOnFlash) return false; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    DATA_FS.remove(to); // rename won't overwrite an existing target on some FS impls
    return DATA_FS.rename(from, to);
}

bool sdMgrFindTileEntry(uint32_t tileId, TileIndexEntry *outEntry) {
    if (!mounted || !outEntry) return false;
    SdLock lock;

    // Fast path: in-PSRAM array
    if (tileIndex) {
        int lo = 0, hi = tileIndexCount - 1;
        while (lo <= hi) {
            int mid = lo + (hi - lo) / 2;
            if (tileIndex[mid].tileId == tileId) {
                memset(outEntry, 0, sizeof(*outEntry));
                outEntry->tileId = tileIndex[mid].tileId;
                outEntry->fileOffset = tileIndex[mid].fileOffset;
                outEntry->fileSize = tileIndex[mid].fileSize;
                return true;
            } else if (tileIndex[mid].tileId < tileId) {
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }
        return false;
    }

    // Direct file-based binary search on /speedmap/index.bin (0 KB PSRAM)
    if (!ensureSdMmcBegun()) return false;
    File f = DATA_FS.open("/speedmap/index.bin", FILE_READ);
    if (!f) return false;

    int lo = 0, hi = tileIndexCount - 1;
    bool found = false;
    while (lo <= hi) {
        int mid = lo + (hi - lo) / 2;
        if (!f.seek((uint32_t)mid * sizeof(TileIndexEntry))) break;
        TileIndexEntry ent;
        if (f.read((uint8_t *)&ent, sizeof(ent)) != sizeof(ent)) break;
        if (ent.tileId == tileId) {
            *outEntry = ent;
            found = true;
            break;
        } else if (ent.tileId < tileId) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    f.close();
    return found;
}

// Returns a pointer to a static buffer, valid until the next call (the only
// caller, sdMgrGetSegmentRoadName, copies it immediately). Two small SD reads:
// the u32 offset, then up to 95 bytes of the NUL-terminated UTF-8 string.
const char *sdMgrGetRoadName(uint32_t nameId) {
    static char buf[96];
    buf[0] = '\0';
    if (nameId == 0 || nameId > roadNameCount) return buf;
    uint32_t offset = 0;
    if (!sdMgrReadBytes("/speedmap/names.bin", roadNameOffsetsBase + (nameId - 1) * 4, (uint8_t *)&offset, 4) ||
        offset >= roadNamePoolSize)
        return buf;
    size_t n = roadNamePoolSize - offset;
    if (n > sizeof(buf) - 1) n = sizeof(buf) - 1;
    if (!sdMgrReadBytes("/speedmap/names.bin", roadNamePoolBase + offset, (uint8_t *)buf, n)) {
        buf[0] = '\0';
        return buf;
    }
    buf[n] = '\0'; // strings are NUL-terminated in the pool; this bounds a corrupt one
    return buf;
}

// Copy up to dstCap-1 bytes of a UTF-8 string, but never split a multi-byte
// sequence: if the byte at the cut point is a UTF-8 continuation byte (10xxxxxx),
// back up to the start of that character. Vietnamese letters are 2-3 UTF-8 bytes,
// so a naive strncpy at a fixed byte length can leave a half character that
// renders as a tofu/garbage glyph (or upsets LVGL's UTF-8 decoder). Always
// NUL-terminates.
static void utf8SafeCopy(char *dst, size_t dstCap, const char *src) {
    if (dstCap == 0) return;
    size_t max = dstCap - 1;
    size_t len = 0;
    while (src[len] != '\0' && len < max) len++;
    if (src[len] != '\0') {
        // We stopped at the cap, not the string end — don't cut mid-character.
        while (len > 0 && ((unsigned char)src[len] & 0xC0) == 0x80) len--;
    }
    memcpy(dst, src, len);
    dst[len] = '\0';
}

const char *sdMgrGetSegmentRoadName(uint32_t segId) {
    static uint32_t sLastSegId = 0xFFFFFFFFu;
    static char sLastRoadName[64] = {0};

    if (segId == 0) return "";
    if (segId == sLastSegId) return sLastRoadName;

    // seg_names.bin is a headerless array indexed by segId; each entry is
    // segNameWidth bytes (2 on legacy v1 cards, 4 on v2 — see the names.bin
    // loader). Read exactly that width into a uint32 name id.
    uint32_t nameId = 0;
    bool ok = sdMgrReadBytes("/speedmap/seg_names.bin", (uint32_t)(segId * segNameWidth),
                             (uint8_t *)&nameId, segNameWidth);
    if (ok && nameId > 0) {
        const char *name = sdMgrGetRoadName(nameId);
        if (name && name[0]) {
            sLastSegId = segId;
            utf8SafeCopy(sLastRoadName, sizeof(sLastRoadName), name);
            return sLastRoadName;
        }
    }

    sLastSegId = segId;
    sLastRoadName[0] = '\0';
    return "";
}

// ---------------------------------------------------------------------------
// Data-install primitives (Phone Update Bridge / online update, 2026-09-26).
// See src/update/DataInstaller.cpp for the staging + journal protocol these
// serve. All are mutex-guarded like the rest of this module.
// ---------------------------------------------------------------------------

// ONE streaming writer at a time, kept OPEN between chunks. The old
// open-append-close-per-chunk pattern (sdMgrAppendBytes) re-walks the FAT
// cluster chain on every open, so a multi-MB file got slower with every chunk;
// holding the handle makes each write O(chunk). The SD mutex is still taken
// only per write, so the map matcher keeps reading tiles between chunks.
static File sWriter;

bool sdMgrWriterOpen(const char *path, bool append) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    if (gDataOnFlash) return false; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    if (sWriter) sWriter.close();
    sWriter = DATA_FS.open(path, append ? FILE_APPEND : FILE_WRITE);
    return (bool)sWriter;
}

bool sdMgrWriterWrite(const uint8_t *buf, size_t len) {
    SdLock lock;
    if (!sWriter) return false;
    return sWriter.write(buf, len) == len;
}

void sdMgrWriterClose() {
    SdLock lock;
    if (sWriter) {
        sWriter.flush();
        sWriter.close();
    }
}

int64_t sdMgrFileSize(const char *path) {
    SdLock lock;
    if (!ensureSdMmcBegun() || !DATA_FS.exists(path)) return -1; // exists() first: open() of a missing file logs an error
    File f = DATA_FS.open(path, FILE_READ);
    if (!f) return -1;
    int64_t s = (int64_t)f.size();
    f.close();
    return s;
}

bool sdMgrExists(const char *path) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    return DATA_FS.exists(path);
}

bool sdMgrMkdir(const char *path) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    if (gDataOnFlash) return false; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    return DATA_FS.exists(path) || DATA_FS.mkdir(path);
}

// Plain rename that does NOT delete an existing target first (FAT rename fails
// if the target exists — the installer journal handles the backup step itself,
// so a crash can never leave BOTH files missing).
bool sdMgrMove(const char *from, const char *to) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    if (gDataOnFlash) return false; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    return DATA_FS.rename(from, to);
}

uint64_t sdMgrFreeBytes() {
    SdLock lock;
    if (!ensureSdMmcBegun()) return 0;
    uint64_t total = dataFsTotalBytes(), used = dataFsUsedBytes();
    return total > used ? total - used : 0;
}

// Replace a small file's whole content (manifest, journal, session record).
bool sdMgrWriteSmallFile(const char *path, const void *data, size_t len) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return false;
    if (gDataOnFlash) return false; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    File f = DATA_FS.open(path, FILE_WRITE);
    if (!f) return false;
    size_t w = f.write((const uint8_t *)data, len);
    f.flush();
    f.close();
    return w == len;
}

// Stream a whole file through SHA-256 with one open handle (readback verify of
// what is ACTUALLY on the card). Mutex taken per 4 KB chunk; tick() (may be
// NULL) is called between chunks — callers pass a watchdog feed.
bool sdMgrSha256File(const char *path, uint8_t out[32], void (*tick)()) {
    uint8_t *buf = (uint8_t *)heap_caps_malloc(4096, MALLOC_CAP_SPIRAM);
    if (!buf) return false;
    File f;
    {
        SdLock lock;
        if (ensureSdMmcBegun()) f = DATA_FS.open(path, FILE_READ);
    }
    if (!f) {
        heap_caps_free(buf);
        return false;
    }
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    bool ok = true;
    for (;;) {
        int r;
        {
            SdLock lock;
            r = f.read(buf, 4096);
        }
        if (r < 0) { ok = false; break; }
        if (r == 0) break;
        mbedtls_sha256_update(&ctx, buf, r);
        if (tick) tick();
    }
    {
        SdLock lock;
        f.close();
    }
    mbedtls_sha256_finish(&ctx, out);
    mbedtls_sha256_free(&ctx);
    heap_caps_free(buf);
    return ok;
}

// Delete every regular file directly inside dir (non-recursive). Used to wipe
// the data-install staging area. Returns files removed, -1 if no card.
int sdMgrClearDir(const char *dir) {
    SdLock lock;
    if (!ensureSdMmcBegun()) return -1;
    if (gDataOnFlash) return -1; // P4 flash fallback is read-only map data (see ensureSdMmcBegun)
    File d = DATA_FS.open(dir);
    if (!d || !d.isDirectory()) return 0;
    // Collect first, delete after: deleting while iterating is unreliable on SD_MMC.
    static char names[48][48];
    int n = 0;
    for (File f = d.openNextFile(); f && n < 48; f = d.openNextFile()) {
        if (!f.isDirectory()) {
            const char *nm = f.name();
            const char *base = strrchr(nm, '/');
            snprintf(names[n++], sizeof(names[0]), "%s/%s", dir, base ? base + 1 : nm);
        }
        f.close();
    }
    d.close();
    int removed = 0;
    for (int i = 0; i < n; i++)
        if (DATA_FS.remove(names[i])) removed++;
    return removed;
}

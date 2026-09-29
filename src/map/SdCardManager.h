#pragma once
#include "SpeedMapFormat.h"
#include <stddef.h> // size_t — sdMgrReadFileChunk()'s offset/length parameters

// Sole owner of the microSD card (spec section 19: "Không để UI / MapMatcher
// / SpeedLimitManager tự ý mở/đóng file SD" — nothing else in the firmware
// opens an SD file directly). SpeedLimitManager.cpp is the only caller.
// Plain functions with an `sdMgr` prefix, not a class — same convention
// every other module in this project uses (gnssTaskStart(),
// webPortalInit()...), kept consistent rather than introducing this
// codebase's first namespace/singleton-object for what's really just one
// more owned-resource module like the others.
//
// Was "not thread-safe by design, only ever called from SpeedLimitManager's
// own task" until log/TripLogger.cpp (added 2026-09-16) became a second real
// caller running on its OWN FreeRTOS task. SD_MMC is one physical peripheral
// shared by both, so every function below is now internally mutex-protected
// (see SdCardManager.cpp's SdLock) — callers still don't need to think about
// locking, same as before, it's just no longer actually safe to skip.

// Mounts the card and validates metadata.bin (magic, format version,
// index.bin CRC) — does NOT read tiles yet (that's lazy, per-lookup, via
// sdMgrReadTile()). Returns false for anything that stops the map feature
// from working: no card, wrong format version, bad CRC, missing files.
// Never crashes or blocks indefinitely on a bad card — every failure path
// here is a plain `return false` plus one `Serial.println`, and the caller
// falls back to reporting UNKNOWN (spec sections 31, 35, 36).
bool sdMgrMount();
// The filesystem holding /speedmap and /triplog: the microSD card, or on the
// P4 board the internal-flash FAT fallback when no card is fitted.
namespace fs { class FS; }
fs::FS &sdMgrDataFs();
bool sdMgrDataOnFlash();

bool sdMgrIsAvailable(); // true only after a successful sdMgrMount()

// Fills `out` with the already-validated metadata (cheap: sdMgrMount()
// already read it into a static buffer). Returns false if !sdMgrIsAvailable().
bool sdMgrGetMetadata(SpeedMapMetadata &out);

// Finds a TileIndexEntry by tileId, supporting both PSRAM array and on-demand file index.
bool sdMgrFindTileEntry(uint32_t tileId, TileIndexEntry *outEntry);

// Points `*out` at the in-RAM CameraPoint array loaded once at
// sdMgrMount() time and sets *outCount — 0 (not a failure) when the card's
// /speedmap/cameras.bin is missing or empty, since camera data is OPTIONAL
// (see CameraPoint's own SpeedMapFormat.h comment): an older card built
// before this feature existed still works for everything else. Unlike
// the tile index, this doesn't require
// sdMgrIsAvailable() (a full, validated speedmap) — cameras.bin is loaded
// independently of metadata.bin/index.bin's own validation, so a region
// extract that for whatever reason has cameras but no tiles (or vice
// versa) still gets whatever it does have.
//
// In-RAM form (2026-09-26): the nationwide data grew to ~100k cameras + ~135k
// signs (4.7 MB as file records), which exhausted PSRAM — the map canvas then
// failed to allocate and the device boot-looped on an LVGL assert. The loader
// keeps only the fields the firmware reads, in compact records, drops signs.bin's
// duplicate camera rows (type 4, already in cameras.bin), and SORTS both arrays
// by latitude so lookups scan a narrow band (pointsLatBegin) instead of every
// point every tick.
#pragma pack(push, 1)
struct CamPt {
    int32_t latE7;
    int32_t lonE7;
    int16_t speedLimitKmh; // -1 = unknown
    uint16_t directionDeg; // 0xFFFF = unknown
};
struct SignPt {
    int32_t latE7;
    int32_t lonE7;
    uint16_t directionDeg; // 0xFFFF = omnidirectional
    uint8_t signType;      // TrafficSignType
    uint8_t speedLimitKmh; // 0 = none
    uint8_t subType;       // 0 = start, 1 = end
};
#pragma pack(pop)

bool sdMgrGetCameras(const CamPt **out, int *outCount);

// Points `*out` at the in-RAM sign array loaded at boot from /speedmap/signs.bin
bool sdMgrGetSigns(const SignPt **out, int *outCount);

// First index whose latE7 >= latE7 in a latitude-sorted point array.
template <class T> inline int pointsLatBegin(const T *a, int n, int32_t latE7) {
    int lo = 0, hi = n;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (a[mid].latE7 < latE7) lo = mid + 1;
        else hi = mid;
    }
    return lo;
}

// Returns the street name for a given segment id (e.g. "Đ. Nguyễn Trãi", "QL 1A")
// or empty string if not available.
const char *sdMgrGetSegmentRoadName(uint32_t segId);

// Returns the street name for a given nameId (1-based index)
const char *sdMgrGetRoadName(uint32_t nameId);

// Reads one tile's segments out of the single packed /speedmap/tiles.bin
// blob (format V2, 2026-09-16 — see SpeedMapFormat.h's own history note on
// why V1's one-file-per-tile layout was abandoned) at `entry.fileOffset`,
// `entry.fileSize` bytes, into `outBuf` (caller-owned, `maxSegments`
// capacity). Returns how many RoadSegment records were actually read via
// `outCount`. The caller (SpeedLimitManager.cpp's getOrLoadTile()) already
// has `entry` from its own index lookup, so this never re-searches the
// index. Returns false if the seek/read fails — SpeedLimitManager.cpp
// treats that tile as empty (no candidates), not as a fatal error.

// Reads raw bytes from any file on SD with mutex protection
bool sdMgrReadBytes(const char *path, uint32_t offset, uint8_t *outBuf, size_t len);

bool sdMgrReadTile(const TileIndexEntry &entry, RoadSegment *outBuf, int maxSegments, int *outCount);

// Appends one line (a trailing '\n' is added) to a text file, creating the
// file and any missing parent directory if needed. Added 2026-09-16 for
// log/TripLogger.cpp — the trip-log CSV is the one thing in this project
// that writes to the SD card rather than only reading it, so this stays
// here (this module's sole-SD-owner rule, see the header comment above)
// instead of TripLogger.cpp opening its own File handle. Works even when
// sdMgrIsAvailable() is false (no speedmap database on the card, or
// sdMgrMount() hasn't run yet / failed its metadata.bin validation) — this
// lazily brings up the underlying SD_MMC peripheral itself on first use if
// nothing has yet (see SdCardManager.cpp's ensureSdMmcBegun()), so
// TripLogger's own task doesn't need to race SpeedLimitManager's task to
// mount the card first. Only actually fails if there's no card / it can't
// be brought up at all, or the file itself can't be opened for append.
bool sdMgrAppendLine(const char *path, const char *line);

// Dev-only diagnostic utility (added 2026-09-21 to retrieve and analyze a
// real road-test's trip log): streams an existing file's raw contents to
// Serial, wrapped in machine-parseable BEGIN/END markers, so a trip-log CSV
// can be pulled over the same serial link the rest of this project's dev
// loop already uses instead of physically pulling the microSD card. No
// automatic caller in the shipped firmware — invoke ad hoc (a temporary
// call from main_ui_demo.cpp, same pattern as this project's other
// TEMP-verification hacks) whenever a future test drive's log needs
// retrieving the same way.
bool sdMgrDumpFileToSerial(const char *path);

// Same dev-only-utility status as sdMgrDumpFileToSerial() above — prints
// one summary line (size, line count, max ego speed seen, whether any
// active target/event was ever logged) for a whole range of trip-log
// session files, so the real road-test session can be picked out from
// among many short/stationary bench-test sessions without dumping each
// one's full content first.
void sdMgrSummarizeTripLogs(uint32_t firstSessionId, uint32_t lastSessionId);

// Enumerates the trip-log session files actually present on the card into
// caller-owned arrays (ids and byte sizes, newest-last), returning how many
// were filled in — backs net/WebPortal.cpp's /triplog page so a finished
// drive's CSV can be listed and downloaded over the device's own WiFi AP
// instead of pulling the microSD card or dumping it over serial (added
// 2026-09-21 after doing exactly that the hard way).
int sdMgrListTripLogs(uint32_t *outIds, uint32_t *outSizes, int maxCount);

// Reads up to bufSize bytes of `path` starting at `offset` into `buf`,
// returning the byte count (0 at EOF, -1 if the file can't be opened).
// Chunked-with-offset rather than handing a File handle back to the
// caller: this module's whole point is that nothing else opens SD files
// directly (and its mutex is held only for the duration of each call, never
// across a whole slow HTTP response).
int sdMgrReadFileChunk(const char *path, size_t offset, uint8_t *buf, size_t bufSize);

// Deletes every /triplog/session_*.csv (mutex-guarded like the rest of this
// module). Returns the number deleted, or -1 if the card isn't available.
// Used by the web control panel's "Clear trip logs" action (net/WebPortal.cpp).
int sdMgrDeleteAllTripLogs();

// Mutex-guarded raw file helpers for the online data updater (net/DataUpdater.cpp).
// sdMgrAppendBytes opens-appends-closes per call (SD mutex held one chunk at a
// time, so the matcher can still read between chunks). Returns true on full write.
bool sdMgrAppendBytes(const char *path, const uint8_t *buf, size_t len);
bool sdMgrRemove(const char *path);              // true if gone (incl. already-absent)
bool sdMgrRename(const char *from, const char *to); // removes an existing target first

// ---- Data-install primitives (Phone Update Bridge / online update) ----
// See src/update/DataInstaller.cpp. One streaming writer at a time.
bool sdMgrWriterOpen(const char *path, bool append);
bool sdMgrWriterWrite(const uint8_t *buf, size_t len);
void sdMgrWriterClose();
int64_t sdMgrFileSize(const char *path);   // -1 if missing
bool sdMgrExists(const char *path);
bool sdMgrMkdir(const char *path);         // true if it exists afterwards
bool sdMgrMove(const char *from, const char *to); // plain rename, never deletes the target
uint64_t sdMgrFreeBytes();
bool sdMgrWriteSmallFile(const char *path, const void *data, size_t len); // truncate + write
bool sdMgrSha256File(const char *path, uint8_t out[32], void (*tick)());  // readback hash
int sdMgrClearDir(const char *dir);             // remove all files in dir (non-recursive)

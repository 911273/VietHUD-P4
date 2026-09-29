#pragma once
#include <Arduino.h>
#include <math.h> // isfinite() — see sanitizeConfig() below

// User-tunable settings, persisted to NVS (core/NvsStore.cpp) and edited
// from ui/Settings.cpp and net/WebPortal.cpp's /api/v1/config. Trimmed
// 2026-09-21 to VietHUD's actual scope (GPS-only offline speed-limit/
// camera/sign warnings) when radar was removed entirely — every field that
// only ever fed the HLK-LD2451/target tracking/TTC risk engine (max range,
// target count, TTC thresholds, audio-gate hysteresis, lane width, tracking
// smoothing, per-target display toggles, demo mode) is gone with it rather
// than left as dead config nobody reads; see git history if any of that is
// ever needed again.
struct AppConfig {
    float brightness = 100; // % — drives the real backlight PWM
    // Gates the vehicle being continuously STATIONARY (see
    // ui/Dashboard.cpp's burn-in-mitigation header comment and
    // gnss/GNSS.h's gnssMsSinceStationary()). 0 = off.
    float autoDimMin = 3;

    // Master alert-audio toggle (tone chimes AND voice cues, see
    // audio/AudioPlayer.h) — user can mute camera/sign/speed-limit warnings
    // without touching device volume. Was originally the radar audio-gate's
    // own on/off switch; repurposed 2026-09-21 to the same role for the
    // sign/camera alerts that replaced radar, no behavior change to the
    // switch itself (still a plain on/off).
    bool audioEnabled = true;

    // Per-alert-type sound (2026-09-26, "cai dat loai canh bao se duoc thong
    // bao bang am thanh"). Each gates BOTH the chime and the voice line of that
    // alert; the visual card/sign is unaffected. audioEnabled above is still the
    // master switch (also toggled by holding the Dashboard ~2.5 s).
    bool audioOverspeed = true;  // over the limit
    bool audioCamera = true;     // speed camera ahead
    bool audioLimitAhead = true; // speed-limit change ahead
    bool audioResident = true;   // residential area start/end
    bool audioNoOvertake = true; // no-overtaking start/end
    bool audioToll = true;       // toll booth
    bool audioLight = true;      // traffic light
    bool audioDanger = true;     // hazard / danger zone
    bool audioSystem = true;     // GPS ready/lost + device-temperature chimes

    // Speaker volume 0-100% (audio/AudioPlayer.cpp audioSetVolume). Defaults to
    // 100 = loudest the NS4168 + full-scale digital path allow (raised from the
    // old hard-coded 80% default 2026-09-24). Applied in applyConfig(),
    // adjustable in Settings > Display and web portal (Cài đặt).
    float audioVolume = 100;

    // Real GNSS calibration (gnss/GNSS.cpp reads these directly every tick —
    // cross-task read without a lock is safe here: a single float/bool field
    // has no torn/garbage intermediate value to race on). spec section
    // 5.3/16.6: speed filter and fix-loss handling must be tunable, not
    // hardcoded, once real hardware exists to tune against.
    float gnssSpeedFilterAlpha = 0.30f; // EMA weight on the new sample; higher = less smoothing, more lag
    float gnssFixTimeoutS = 3.0f;       // no fresh fix for this long -> report GNSS lost (spec T13)
    // GPS speed calibration (user-requested 2026-09-22, "hieu chinh toc do
    // GPS") — a real vehicle's own speedometer and a GPS-derived speed
    // rarely agree exactly (tire wear/size, GPS's own small systematic
    // error), so this lets the driver nudge the displayed/logged speed to
    // match their speedometer. A PERCENTAGE, not a flat km/h offset: the
    // real-world mismatch this corrects for (tire circumference, etc.) scales
    // with speed rather than being a fixed number of km/h at every speed.
    // Applied in gnss/GNSS.cpp to the raw module reading, before the
    // median+EMA filter, so both rawSpeedKmh and the filtered egoSpeedKmh
    // (and everything downstream: the Dashboard, overspeed logic, map
    // matching's heading-gate speed check) see the corrected value — there is
    // no separate "true" vs "displayed" speed anywhere else in this project.
    float gnssSpeedCalibrationPct = 0.0f;
    float overspeedOffsetKmh = 1.0f; // overspeed warning fires when egoSpeed > limit + this (km/h). Clamp 0..10; user-tunable in Settings > Sensors and web portal (Cài đặt).

    // Ahead-warning lookahead/trigger distances (user-requested 2026-09-22)
    // — both used to be fixed constants in map/SpeedLimitManager.cpp
    // (kAheadLookaheadM/kCameraWarnDistanceM); moved here so they're tunable
    // per-driver (a highway driver wants more warning distance at speed than
    // someone doing city traffic) without a firmware rebuild. Sign warnings
    // (resident-area/no-overtaking/toll/traffic-light) keep their own
    // separate fixed distance in SpeedLimitManager.cpp — not requested to be
    // configurable, and giving every alert type its own slider would clutter
    // Settings for little real benefit given they're all in the same
    // 300-400m ballpark already.
    //
    // Both capped at 100m max / 100m default (user-requested 2026-09-22,
    // "khoang cach toi da de canh bao la 100m, mac dinh la 100m") — was
    // 50-300m/100m and 100-800m/300m respectively; camera's default dropped
    // from 300 to 100 to match. Floor stays at 50m on both (not also pulled
    // down to 100) so the slider still has real room to move rather than
    // collapsing to a single fixed value.
    float aheadLimitWarnDistM = 100.0f; // how far ahead to project + re-match for an upcoming speed-limit CHANGE
    float cameraWarnDistM = 100.0f;     // how far out an upcoming speed camera starts showing on the alert card

    // Trip logging (log/TripLogger.cpp) — periodic + event-triggered CSV log
    // to the microSD card, for reviewing a drive's speed-limit/camera/sign
    // warnings afterward instead of only road-testing by feel. Defaults ON:
    // logging is a passive, fail-open diagnostic (no effect on safety logic
    // either way), but this switch exists for anyone who'd rather not
    // accumulate files on their card.
    bool tripLoggingEnabled = true;

    // Local WiFi AP credentials (net/WebPortal.cpp) — plain char arrays, not
    // floats, so sanitizeConfig()/clampConfig() below don't touch them (both
    // only cover numeric fields — see their own "named field-by-field"
    // comments). Only the credentials persist; WiFi's on/off state does NOT
    // — it always starts OFF at boot regardless of what it was last session
    // (user-requested 2026-09-14: "Mặc định là wifi tắt"), toggled at
    // runtime only, via a 3s+ hold on the Dashboard or the switch in
    // Settings > WiFi — see net/WebPortal.h. Defaults renamed 2026-09-21
    // (radar_car -> VietHUD product rename) — no functional change.
    char wifiSsid[32] = "VietHUD";
    char wifiPassword[64] = "12345678"; // WPA2 needs >=8 chars — see WebPortal.cpp's applyWifiState() fallback

    // Optional STATION credentials (net/WebPortal.cpp, added 2026-09-25): when
    // staSsid is non-empty the device ALSO joins this network (AP+STA mode) to
    // get internet — used for NTP time sync (accurate clock without waiting for
    // a GPS fix), and a base for future online updates. Empty staSsid = AP-only,
    // as before. Editable in web portal (Cài đặt). Never auto-enables anything on its own.
    char staSsid[32] = "";
    char staPassword[64] = "";

    // WiFi Manager (2026-09-26): a small list of REMEMBERED station networks.
    // When the user turns WiFi on, the web task scans and connects to the
    // strongest SAVED network currently in range, rotating to the next candidate
    // on failure (net/WebPortal.cpp). staSsid/staPassword above mirror whichever
    // network is currently active — they're also how a pre-manager single-STA
    // config is migrated in (NvsStore.cpp seeds savedNetworks[0] from them once).
    // WiFi still starts OFF at boot; the manager only runs after a manual enable.
    static const int kMaxSavedNetworks = 5;
    struct WifiNetwork {
        char ssid[32] = "";
        char password[64] = "";
    };
    WifiNetwork savedNetworks[kMaxSavedNetworks];
    int savedNetworkCount = 0;

    // Base URL the online data updater fetches from (net/DataUpdater.cpp, added
    // 2026-09-25). Points at the Raspberry Pi's public endpoint that serves the
    // map/warning data + manifest.txt (Pi bridges Google Drive via rclone).
    // e.g. GitHub raw CDN. Must end with "/". Editable in web portal (Cài đặt) and
    // Settings > WiFi. Default = the project's public GitHub release path so OTA
    // works out of the box (GitHub raw 301/302s to Fastly — DataUpdater follows
    // redirects). Empty = updater disabled.
    char dataUpdateUrl[128] = "https://raw.githubusercontent.com/911273/VietHUD/main/speedmap/";
    // Auto-off the WiFi AP after this many minutes with NO client connected.
    // Always on (1..120 min, default 10 — user rule 2026-09-26: WiFi is OFF at
    // boot and must switch itself off again when unused; the old 0 = "never"
    // is gone). Saves power/heat/exposure on a windscreen device left with
    // WiFi on. web portal (Cài đặt) + Settings; see WebPortal.cpp webTaskFn().
    float wifiAutoOffMin = 10;

    // Display settings (user-requested 2026-09-15). Both are floats used as
    // small enums — no dropdown widget exists in Settings.cpp, only sliders.
    //
    // screenRotation matches Arduino_GFX's own 0-3 rotation values exactly
    // (display/DisplayDriver.cpp passes this straight through to
    // Arduino_Canvas, and touch/TouchTask.cpp to AXS15231BTouch — both use
    // the identical convention, confirmed by reading each driver's rotation
    // switch). Default 1 = this project's boot rotation ever since
    // dispcfg.h's DEMO_ROTATION was introduced — changing this does NOT
    // take effect live (see ui/Settings.cpp's rotation row): both Dashboard
    // and Settings are laid out once at boot for whichever orientation is
    // active then, so a change only applies on the next restart.
#ifdef VIETHUD_P4
    float screenRotation = 1; // P4 board (480x800 portrait panel) is mounted landscape
#else
    float screenRotation = 0;
#endif
    // themeMode: 0=Auto (today's only behavior — ui/Dashboard.cpp's
    // applyTheme() follows gnss.daytime's real sunrise/sunset calculation),
    // 1=Light, 2=Dark (both override gnss.daytime rather than replacing the
    // calculation — GNSS.cpp's sunrise/sunset math is untouched and still
    // runs regardless, since the sun icon and the local-time-from-longitude
    // estimate both still need it). Applies live, no restart needed.
    float themeMode = 0;
    // Backlight mode (2026-09-26): 0 = Auto — at night (GNSS sunrise/sunset
    // calc, same gnss.daytime as the Auto theme) the backlight is capped at
    // kNightBrightnessPct; by day it uses `brightness`. 1 = Manual — always
    // `brightness`. Float-as-enum like themeMode (choice row in Settings).
    float brightnessMode = 0;
    static constexpr float kNightBrightnessPct = 50.0f;
    // Map display (vector only — the raster JPEG background was removed
    // 2026-09-26; vector roads are always drawn).
    bool showVehicleTrail = true;
    // Heading-up map rotation (2026-09-24). true = the whole map rotates so the
    // travel direction is always at 12 o'clock; false = north-up (map fixed,
    // north up) — the simpler, proven mode, and a fallback if rotation
    // misbehaves on a given panel.
    bool mapHeadingUp = true;
    // Fallback speed limit shown when the position genuinely can't resolve a
    // limit (no GPS fix's road match, off the mapped network). 0 = off (show
    // "--"). Default 50 = Vietnam's baseline urban limit (user-requested
    // 2026-09-24). Only applied with a real fix; source is marked DEFAULT.
    float defaultLimitKmh = 50.0f;
};

// Single shared instance, defined in main_ui_demo.cpp. NOTE: read without a
// lock from other tasks — a conscious, scoped simplification, not the full
// mutex-protected ConfigStore design in docs/V1.2_hardening_proposal.md
// section A.2. Every field is an independently-aligned float/bool/char[], so
// a cross-task read during a Settings write can only ever see one field's
// old-or-new value, never a torn/garbage one — there is no multi-field
// invariant here that a reader depends on being atomic.
extern AppConfig cfg;

inline void clampConfig(AppConfig &c) {
    c.brightness = constrain(c.brightness, 5.0f, 100.0f);
    c.autoDimMin = constrain(c.autoDimMin, 0.0f, 30.0f);
    c.gnssSpeedFilterAlpha = constrain(c.gnssSpeedFilterAlpha, 0.05f, 0.90f);
    c.gnssFixTimeoutS = constrain(c.gnssFixTimeoutS, 1.0f, 10.0f);
    c.gnssSpeedCalibrationPct = constrain(c.gnssSpeedCalibrationPct, -15.0f, 15.0f);
    c.overspeedOffsetKmh = constrain(c.overspeedOffsetKmh, 0.0f, 10.0f);
    c.defaultLimitKmh = constrain(c.defaultLimitKmh, 0.0f, 120.0f);
    c.audioVolume = constrain(c.audioVolume, 0.0f, 100.0f);
    c.wifiAutoOffMin = constrain(c.wifiAutoOffMin, 1.0f, 120.0f);
    c.aheadLimitWarnDistM = constrain(c.aheadLimitWarnDistM, 50.0f, 100.0f);
    c.cameraWarnDistM = constrain(c.cameraWarnDistM, 50.0f, 100.0f);
    c.screenRotation = constrain(c.screenRotation, 0.0f, 3.0f);
    c.themeMode = constrain(c.themeMode, 0.0f, 2.0f);
    c.brightnessMode = constrain(c.brightnessMode, 0.0f, 1.0f);
    if (c.savedNetworkCount < 0) c.savedNetworkCount = 0;
    if (c.savedNetworkCount > AppConfig::kMaxSavedNetworks) c.savedNetworkCount = AppConfig::kMaxSavedNetworks;
}

// Replaces any non-finite (NaN/Inf) field with AppConfig's own default —
// guards against a corrupted NVS flash page (bit rot, power loss mid-write)
// producing a garbage float bit pattern. Deliberately checked BEFORE
// clampConfig(): constrain(NaN, lo, hi) returns NaN (every comparison
// against NaN is false), so clamping alone can't catch this. Named
// field-by-field (not a generic byte/float scan over the whole struct)
// because AppConfig mixes float/bool/char[] fields — bools/strings can't be
// NaN, and a blind reinterpret_cast<float*> scan would misread their bytes
// and misalign every float field after them.
inline void sanitizeConfig(AppConfig &c) {
    static const AppConfig d;
    if (!isfinite(c.brightness)) c.brightness = d.brightness;
    if (!isfinite(c.autoDimMin)) c.autoDimMin = d.autoDimMin;
    if (!isfinite(c.gnssSpeedFilterAlpha)) c.gnssSpeedFilterAlpha = d.gnssSpeedFilterAlpha;
    if (!isfinite(c.gnssFixTimeoutS)) c.gnssFixTimeoutS = d.gnssFixTimeoutS;
    if (!isfinite(c.gnssSpeedCalibrationPct)) c.gnssSpeedCalibrationPct = d.gnssSpeedCalibrationPct;
    if (!isfinite(c.overspeedOffsetKmh)) c.overspeedOffsetKmh = d.overspeedOffsetKmh;
    if (!isfinite(c.defaultLimitKmh)) c.defaultLimitKmh = d.defaultLimitKmh;
    if (!isfinite(c.audioVolume)) c.audioVolume = d.audioVolume;
    if (!isfinite(c.wifiAutoOffMin)) c.wifiAutoOffMin = d.wifiAutoOffMin;
    if (!isfinite(c.aheadLimitWarnDistM)) c.aheadLimitWarnDistM = d.aheadLimitWarnDistM;
    if (!isfinite(c.cameraWarnDistM)) c.cameraWarnDistM = d.cameraWarnDistM;
    if (!isfinite(c.screenRotation)) c.screenRotation = d.screenRotation;
    if (!isfinite(c.themeMode)) c.themeMode = d.themeMode;
    if (!isfinite(c.brightnessMode)) c.brightnessMode = d.brightnessMode;
}

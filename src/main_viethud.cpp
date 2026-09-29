// VietHUD — GPS-only offline speed-limit/camera/traffic-sign warning device.
// Radar (HLK-LD2451) and everything built around it were removed entirely
// 2026-09-21 — this is a full product replacement, not a side-by-side demo
// (see docs/VietHUD_offline_speed_alert_plan.md and README.md). Renamed
// from main_ui_demo.cpp at the same time: this is now the one true app
// entry point, no longer a "UI verification demo" layered on top of
// something else — see git history if the earlier radar/demo-mode content
// is ever needed for reference.
//
// GNSS (u-blox M10N, UART2) has been real since 2026-09-15 — see gnss/GNSS.h
// and include/pincfg.h for the wiring. This build exercises the Dashboard
// layout, the offline microSD speed-limit/camera/sign map matching, and a
// Settings screen against real sensor data end to end.
//
// Settings changes have real, visible effect (they're not a dead mockup):
// the Brightness slider drives the *real* backlight via LEDC PWM, Theme/
// Rotation apply live or on next boot as documented in Settings.cpp, and
// the alert-audio toggle gates both tone chimes and voice playback
// (audio/AudioPlayer.h). Settings persist to NVS (ESP32 Preferences)
// across reboots.
//
// This file is a thin orchestrator (module split 2026-09-14 — see
// docs/V1.2_hardening_proposal.md section A): setup()/loop() wire together
// core/ (config + mutex-protected shared state), display/, gnss/ (GNSS),
// touch/ (TouchTask), net/ (WebPortal — WiFi AP + live telemetry/config/OTA
// over HTTP, added 2026-09-14), map/ (SpeedLimitManager — offline microSD
// speed-limit/camera/sign map matching by GNSS position+heading, added
// 2026-09-15, see map/SpeedMapFormat.h for the binary database spec),
// audio/ (AudioPlayer — tone chimes + queued Vietnamese voice playback), and
// ui/ (Dashboard, Settings). The sensor tasks run independently on Core 0;
// this file's setup()/loop() IS the UI/render task on Core 1 (the default
// Arduino loopTask core).

#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h> // update-mode STA connect (runDataUpdateMode) — see net/DataUpdater.h
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // uxTaskGetStackHighWaterMark() — TEMP RAM-audit diagnostic, see the [mem] block below

#include "core/AppConfig.h"
#include "core/NvsStore.h"
#include "core/SharedState.h"
#include "display/DisplayDriver.h"
#include "gnss/GNSS.h"
#include "gnss/RoadSim.h"
#ifdef VIETHUD_P4
#include "net/PhoneGpsBle.h"
#endif
#include "map/SpeedLimitManager.h"
#include "map/SdCardManager.h"
#include "log/TripLogger.h"
#include "net/WebPortal.h"
#include "net/DataUpdater.h" // dataUpdateStart()/GetStatus() — serial 'u' bench trigger
#include "update/DataInstaller.h" // boot-time atomic data install + rollback (Phone Update Bridge)
#include <esp_ota_ops.h> // esp_ota_mark_app_valid_cancel_rollback() — firmware OTA rollback
#include "touch/TouchTask.h"
#include "ui/Dashboard.h"
#include "ui/Settings.h"
#include "audio/AudioPlayer.h"

AppConfig cfg; // extern-declared in core/AppConfig.h — see the comment there

// VRE boot-logo splash image (src/ui/logo_vre.c, RGB565A8, white keyed to
// transparent) — user-requested 2026-09-24 boot screen.
LV_IMAGE_DECLARE(logo_vre);

// Arduino-ESP32's cores/esp32/main.cpp declares this as a WEAK function
// (default 8192 bytes, see ARDUINO_LOOP_STACK_SIZE there) specifically so a
// sketch can override it — this is that override. Raised to 16384
// 2026-09-22 after a REAL, reproducible crash on hardware: "Guru Meditation
// Error: Core 1 panic'ed (Unhandled debug exception) — Stack canary
// watchpoint triggered (loopTask)" while the (since removed, 2026-09-26)
// raster-map background retried a failed SD read every 2s directly from
// updateMapCanvas() (ui/Dashboard.cpp), itself called every ~150ms tick from
// this file's own loop() — i.e. loopTask, the Arduino default/UI task this
// whole file's setup()/loop() runs as (see this file's own header comment).
// uxTaskGetStackHighWaterMark() had already been observed at a mere 278
// bytes free right before the crash (this file's own [mem] block), so this
// isn't a guess at a hypothetical risk — it's a confirmed near-miss that
// then became a real one. Doubling to 16384 is the standard, minimal-risk
// fix for genuine stack pressure (loopTask is UI-only work, not a
// tightly-budgeted small task like the sensor tasks elsewhere in this
// project). Kept at 16384 after the raster code was removed: the vector map
// draw + SD tile reads still run from this task.
size_t getArduinoLoopTaskStackSize(void) { return 16384; }

static lv_display_t *lvDisplay;
static lv_indev_t *lvTouchIndev;

static uint32_t lastStatsMs = 0;
static uint32_t lastMemStatsMs = 0;

// Bench-only synthetic touch (serial 'T'/'H'/'G' in loop()): lets a PC drive the
// UI without a finger — a press from (x0,y0) to (x1,y1) held until sInjEndMs.
static volatile uint32_t sInjStartMs = 0, sInjEndMs = 0;
static volatile int16_t sInjX0, sInjY0, sInjX1, sInjY1;

static void touch_read_cb(lv_indev_t *, lv_indev_data_t *data) {
    uint32_t nowMs = millis();
    if (sInjEndMs && (int32_t)(nowMs - sInjEndMs) < 0) {
        uint32_t span = sInjEndMs - sInjStartMs, t = nowMs - sInjStartMs;
        data->point.x = sInjX0 + (int32_t)(sInjX1 - sInjX0) * (int32_t)t / (int32_t)(span ? span : 1);
        data->point.y = sInjY0 + (int32_t)(sInjY1 - sInjY0) * (int32_t)t / (int32_t)(span ? span : 1);
        data->state = LV_INDEV_STATE_PRESSED;
        wakeScreen();
        return;
    }
    if (sInjEndMs) { // injected gesture just ended: one release at its end point
        sInjEndMs = 0;
        data->point.x = sInjX1;
        data->point.y = sInjY1;
        data->state = LV_INDEV_STATE_RELEASED;
        Serial.println("[touch] injected UP");
        return;
    }
    TouchPoint p = touchSnapshot();
    static bool wasTouched = false; // only log on DOWN/UP transitions, not every poll
    if (p.pressed) {
        data->point.x = p.x;
        data->point.y = p.y;
        data->state = LV_INDEV_STATE_PRESSED;
        wakeScreen();
        if (!wasTouched) Serial.printf("[touch] DOWN x=%u y=%u\n", p.x, p.y);
        wasTouched = true;
    } else {
        if (wasTouched) Serial.println("[touch] UP");
        wasTouched = false;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ---------------------------------------------------------------------
// Boot splash (user-requested 2026-09-24): show the VRE logo for ~2s with a
// modern motion — scale-in with a slight overshoot + fade-in, a brief hold,
// then a fade-out that hands over to the Dashboard with a cross-fade. Driven
// by a manual pump loop (we're single-threaded in setup() here) so the timing
// is exact and the handover happens precisely at the end. dashboardScreen must
// already be built before this is called (it's the cross-fade target).
static void showBootSplash() {
    lv_obj_t *splash = lv_obj_create(NULL);
    lv_obj_remove_style_all(splash);
    lv_obj_set_style_bg_opa(splash, LV_OPA_COVER, 0);
    // Subtle vertical gradient (deep navy -> near-black) for a modern feel.
    lv_obj_set_style_bg_color(splash, lv_color_hex(0x0A1526), 0);
    lv_obj_set_style_bg_grad_color(splash, lv_color_hex(0x04060A), 0);
    lv_obj_set_style_bg_grad_dir(splash, LV_GRAD_DIR_VER, 0);
    lv_obj_clear_flag(splash, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *logo = lv_image_create(splash);
    lv_image_set_src(logo, &logo_vre);
    lv_obj_center(logo);
    lv_image_set_pivot(logo, logo_vre.header.w / 2, logo_vre.header.h / 2);
    lv_image_set_scale(logo, 128);          // start at 0.5x
    lv_obj_set_style_opa(logo, LV_OPA_TRANSP, 0);
    lv_screen_load(splash);

    const uint32_t IN = 700, HOLD_END = 1550, TOTAL = 2000;
    const float c1 = 1.70158f, c3 = c1 + 1.0f; // ease-out-back overshoot
    uint32_t t0 = millis(), lastTick = t0;
    for (;;) {
        uint32_t now = millis();
        uint32_t el = now - t0;
        int scale; lv_opa_t opa;
        if (el < IN) {
            float p = (float)el / (float)IN;
            float p1 = p - 1.0f;
            float eb = 1.0f + c3 * p1 * p1 * p1 + c1 * p1 * p1; // overshoots ~1.1 then settles to 1
            scale = 128 + (int)((256 - 128) * eb);
            float o = (float)el / 500.0f; if (o > 1.0f) o = 1.0f;
            opa = (lv_opa_t)(255.0f * o);
        } else if (el < HOLD_END) {
            scale = 256; opa = LV_OPA_COVER;
        } else if (el < TOTAL) {
            float q = (float)(el - HOLD_END) / (float)(TOTAL - HOLD_END);
            scale = 256 + (int)(44 * q);                 // gentle grow on the way out
            opa = (lv_opa_t)(255.0f * (1.0f - q));
        } else {
            break;
        }
        lv_image_set_scale(logo, scale);
        lv_obj_set_style_opa(logo, opa, 0);
        lv_tick_inc(now - lastTick); lastTick = now;
        lv_timer_handler();
        esp_task_wdt_reset();
        delay(8);
    }
    // Cross-fade to the Dashboard; auto_del=true frees the splash afterwards.
    lv_screen_load_anim(dashboardScreen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, true);
    // Pump the transition so it actually plays before setup() moves on.
    uint32_t tt = millis(), lt = tt;
    while (millis() - tt < 340) {
        uint32_t now = millis();
        lv_tick_inc(now - lt); lt = now;
        lv_timer_handler();
        esp_task_wdt_reset();
        delay(8);
    }
}

// ---------------------------------------------------------------------
// Shared pump for the blocking setup-time screens below (update mode, install
// result): drive LVGL + feed the watchdog while nothing else is running yet.
static uint32_t sPumpLastMs = 0;
static void setupPump() {
    uint32_t n = millis();
    if (sPumpLastMs == 0) sPumpLastMs = n;
    lv_tick_inc(n - sPumpLastMs);
    sPumpLastMs = n;
    lv_timer_handler();
    esp_task_wdt_reset();
    delay(10);
}

// A simple full-screen status page used by update mode and the install-result
// notice: big icon, title, message, optional progress bar, footer hint.
struct StatusScreen {
    lv_obj_t *scr, *icon, *title, *msg, *bar, *sub, *hint;
};
static StatusScreen makeStatusScreen(const char *titleTxt, uint32_t accent) {
    StatusScreen s;
    s.scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s.scr, lv_color_hex(0x0B0F14), 0);
    lv_obj_clear_flag(s.scr, LV_OBJ_FLAG_SCROLLABLE);
    int w = gfx->width();
    s.icon = lv_label_create(s.scr);
    lv_obj_set_style_text_font(s.icon, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(s.icon, lv_color_hex(accent), 0);
    lv_label_set_text(s.icon, LV_SYMBOL_DOWNLOAD);
    lv_obj_align(s.icon, LV_ALIGN_TOP_MID, 0, 26);
    s.title = lv_label_create(s.scr);
    lv_obj_set_style_text_color(s.title, lv_color_hex(accent), 0);
    lv_label_set_text(s.title, titleTxt);
    lv_obj_align(s.title, LV_ALIGN_TOP_MID, 0, 70);
    s.msg = lv_label_create(s.scr);
    lv_obj_set_width(s.msg, w - 60);
    lv_label_set_long_mode(s.msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(s.msg, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s.msg, lv_color_hex(0xE6EDF3), 0);
    lv_label_set_text(s.msg, "");
    lv_obj_align(s.msg, LV_ALIGN_TOP_MID, 0, 104);
    s.bar = lv_bar_create(s.scr);
    lv_obj_set_size(s.bar, w - 120, 12);
    lv_obj_align(s.bar, LV_ALIGN_TOP_MID, 0, 196);
    lv_obj_set_style_bg_color(s.bar, lv_color_hex(0x1E2A38), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s.bar, lv_color_hex(accent), LV_PART_INDICATOR);
    lv_bar_set_range(s.bar, 0, 100);
    lv_obj_add_flag(s.bar, LV_OBJ_FLAG_HIDDEN);
    s.sub = lv_label_create(s.scr);
    lv_obj_set_style_text_color(s.sub, lv_color_hex(0x8A98A8), 0);
    lv_label_set_text(s.sub, "");
    lv_obj_align(s.sub, LV_ALIGN_TOP_MID, 0, 216);
    s.hint = lv_label_create(s.scr);
    lv_obj_set_style_text_color(s.hint, lv_color_hex(0x6B7888), 0);
    lv_label_set_text(s.hint, "");
    lv_obj_align(s.hint, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_screen_load(s.scr);
    return s;
}

// After a boot that swapped in new data (or rolled back a bad install), tell
// the driver what happened for ~2.5 s before the normal splash.
static void showInstallResult(bool rolledBack) {
    char detail[64];
    installerLastResult(detail, sizeof(detail), false);
    Serial.printf("[install] result screen: %s (%s)\n", rolledBack ? "ROLLED BACK" : "UPDATED", detail);
    StatusScreen s = makeStatusScreen(rolledBack ? "PREVIOUS DATA RESTORED" : "DATA UPDATED",
                                      rolledBack ? 0xE8B931 : 0x34C46A);
    lv_label_set_text(s.icon, rolledBack ? LV_SYMBOL_WARNING : LV_SYMBOL_OK);
    char b[160];
    if (rolledBack)
        snprintf(b, sizeof(b), "The update could not be used (%s).\nVietHUD went back to the previous data.", detail);
    else
        snprintf(b, sizeof(b), "Maps and alerts installed.\nData version: %s", detail);
    lv_label_set_text(s.msg, b);
    for (int i = 0; i < 250; i++) setupPump();
    lv_obj_delete(s.scr);
}

// ---------------------------------------------------------------------
// "Update mode" (Mode A — the device downloads by itself): entered at boot when
// the user asked for a direct online update (web or on-screen), which set an
// NVS flag and rebooted. We land here with ONLY display + LVGL + SD up — none
// of the map/audio/GNSS/AP tasks have started — so a big block of internal RAM
// is free for the TLS handshake the normal running app can't spare (TLS needs
// ~16 KB contiguous). Joins the configured / saved WiFi networks in turn, runs
// the download (which stages + verifies through DataInstaller), then reboots;
// the boot-time installer swaps the data in. This function never returns.
static void runDataUpdateMode() {
    Serial.println("[update-mode] entered — display+SD only, connecting WiFi for OTA");
    StatusScreen s = makeStatusScreen("DATA UPDATE", 0x3DA5FF);
    lv_label_set_text(s.hint, "Do not power off during the update");
    lv_label_set_text(s.msg, "Connecting to Wi-Fi...");
    for (int i = 0; i < 12; i++) setupPump();
    auto finish = [&](const char *text, bool ok, int ticks) {
        lv_label_set_text(s.icon, ok ? LV_SYMBOL_OK : LV_SYMBOL_WARNING);
        lv_obj_set_style_text_color(s.icon, lv_color_hex(ok ? 0x34C46A : 0xE8B931), 0);
        lv_label_set_text(s.msg, text);
        lv_label_set_text(s.hint, "Restarting...");
        for (int i = 0; i < ticks; i++) setupPump();
        ESP.restart();
    };

    // Candidate networks: the active STA creds first, then every saved one.
    const char *ssids[AppConfig::kMaxSavedNetworks + 1];
    const char *passes[AppConfig::kMaxSavedNetworks + 1];
    int nc = 0;
    if (cfg.staSsid[0]) {
        ssids[nc] = cfg.staSsid;
        passes[nc++] = cfg.staPassword;
    }
    for (int i = 0; i < cfg.savedNetworkCount; i++) {
        bool dup = false;
        for (int k = 0; k < nc; k++) dup |= strcmp(ssids[k], cfg.savedNetworks[i].ssid) == 0;
        if (!dup && cfg.savedNetworks[i].ssid[0]) {
            ssids[nc] = cfg.savedNetworks[i].ssid;
            passes[nc++] = cfg.savedNetworks[i].password;
        }
    }
    if (nc == 0) finish("No Internet Wi-Fi set for VietHUD.\nUpdate from a phone instead (Data section).", false, 400);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // keep the radio hot while we wait for the AP to appear
    bool up = false;
    for (int c = 0; c < nc && !up; c++) {
        Serial.printf("[update-mode] connecting to \"%s\"\n", ssids[c]);
        WiFi.disconnect();
        WiFi.begin(ssids[c], passes[c]);
        // iOS Personal Hotspot parks its radio when no client is attached, so the
        // first association attempt can miss even when the phone is "on":
        // re-issue begin() every 12 s within a ~30 s window per network.
        uint32_t t0 = millis(), lastRebegin = t0;
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 30000) {
            if (millis() - lastRebegin > 12000) {
                WiFi.disconnect();
                WiFi.begin(ssids[c], passes[c]);
                lastRebegin = millis();
            }
            char b[96];
            snprintf(b, sizeof(b), "Connecting to Wi-Fi \"%s\"... %lus", ssids[c],
                     (unsigned long)((millis() - t0) / 1000));
            lv_label_set_text(s.msg, b);
            setupPump();
        }
        up = WiFi.status() == WL_CONNECTED;
    }
    if (!up) {
        Serial.printf("[update-mode] WiFi connect FAILED (last status=%d)\n", WiFi.status());
        finish("Could not connect to Wi-Fi.\nTurn on a 2.4 GHz hotspot and retry,\nor update from a phone.", false, 500);
    }
    configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
    Serial.printf("[update-mode] STA up: %s  free internal=%u biggest=%u\n", WiFi.localIP().toString().c_str(),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    lv_label_set_text(s.msg, "Connected. Checking for updates...");
    for (int i = 0; i < 10; i++) setupPump();

    if (!dataUpdateStart()) {
        DataUpdateStatus st = dataUpdateGetStatus();
        char b[112];
        snprintf(b, sizeof(b), "Error: %s", st.message);
        finish(b, false, 450);
    }
    lv_obj_clear_flag(s.bar, LV_OBJ_FLAG_HIDDEN);
    for (;;) {
        DataUpdateStatus st = dataUpdateGetStatus();
        lv_label_set_text(s.msg, st.message);
        if (st.filesTotal > 0) {
            int cur = st.filesDone < st.filesTotal ? st.percent : 0;
            int overall = (st.filesDone * 100 + cur) / st.filesTotal;
            lv_bar_set_value(s.bar, overall, LV_ANIM_OFF);
            char b[48];
            snprintf(b, sizeof(b), "File %d/%d  ·  %d%%",
                     st.filesDone < st.filesTotal ? st.filesDone + 1 : st.filesTotal, st.filesTotal, overall);
            lv_label_set_text(s.sub, b);
        }
        setupPump();
        if (st.state == DU_SUCCESS || st.state == DU_FAILED) {
            Serial.printf("[update-mode] terminal state=%d: %s\n", st.state, st.message);
            finish(st.message, st.state == DU_SUCCESS, 300); // success path: the task reboots first
        }
    }
}

// ---------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[viethud] VietHUD — offline GPS speed-limit/camera/sign warning device");
    // Shared with log/TripLogger.cpp, which persists the same string into
    // every session's CSV — see tripLogResetReasonStr()'s own comment for
    // why that matters (a reset mid-drive has no serial monitor attached).
    Serial.printf("[viethud] last reset reason: %s\n", tripLogResetReasonStr());

    // Last-resort recovery for a genuinely stuck I2C transaction (touch
    // driver's own endTransmission()/requestFrom() error handling + bus
    // reset covers transactions that fail promptly, but a real hardware
    // hang confirmed 2026-09-14 blocked loop() outright with no error ever
    // returned). If loop() doesn't come back around to feed this within 5s,
    // the watchdog panics and reboots instead of freezing forever.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    { // IDF 5: the core already created the TWDT — reconfigure it (10 s, panic)
        esp_task_wdt_config_t wc = {.timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = true};
        esp_task_wdt_reconfigure(&wc);
    }
#else
    esp_task_wdt_init(10, true);
#endif
    esp_task_wdt_add(NULL);

    loadConfigFromNVS(cfg);

    backlightBegin();
    applyConfig(); // sets initial backlight duty from loaded cfg.brightness

    displayBegin();
    touchTaskStart();
    audioInit();
    // Startup music: the power-on jingle (pure tones, always works) plays now,
    // over the ~2s boot splash. The spoken welcome greeting is deferred until
    // AFTER the splash finishes (queued below, right after showBootSplash()).
    audioPlayStartupJingle();

    lv_init();
#ifdef VIETHUD_P4
    // P4: native-resolution display (DIRECT mode, PSRAM frame, PPA rotation
    // into vsync-swapped double framebuffers) — see display/DisplayDriverP4.cpp.
    lvDisplay = displayCreateLvgl();
#else
    lvDisplay = lv_display_create(gfx->width(), gfx->height());
    lv_display_set_flush_cb(lvDisplay, dispFlushCb);

    // LVGL renders every pixel into these buffers, so they belong in internal
    // SRAM — PSRAM is several times slower per access and it showed (and the
    // fast dispFlushCb blit reads them strided, which is only cheap on internal
    // SRAM). 8 rows each = ~15 KB total (reduced from 20 rows/38 KB on
    // 2026-09-25 to free internal RAM for the WiFi driver — esp_wifi_init
    // failed with ESP_ERR_NO_MEM at the old size, and WiFi-on free internal was
    // still down to ~10 KB after a first trim). LVGL partial mode works with any
    // buffer height; smaller just means more flush_cb strips per frame.
    size_t bufBytes = (size_t)gfx->width() * 8 * sizeof(lv_color_t);
    lv_color_t *drawBuf1 = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    lv_color_t *drawBuf2 = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!drawBuf1 || !drawBuf2) {
        Serial.println("[viethud] WARN: internal RAM draw buffers failed, falling back to PSRAM");
        if (drawBuf1) heap_caps_free(drawBuf1);
        if (drawBuf2) heap_caps_free(drawBuf2);
        drawBuf1 = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_SPIRAM);
        drawBuf2 = (lv_color_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_SPIRAM);
    }
    lv_display_set_buffers(lvDisplay, drawBuf1, drawBuf2, bufBytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
#endif

    lvTouchIndev = lv_indev_create();
    lv_indev_set_type(lvTouchIndev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvTouchIndev, touch_read_cb);
    // REVERTED from 10ms 2026-09-14: combined with 400kHz I2C this made the
    // touch controller chronically lock up (see AXS15231BTouch.cpp begin()).
    // 20ms is a middle ground between that and LVGL's laggier ~30ms default.
    lv_timer_set_period(lv_indev_get_read_timer(lvTouchIndev), 20);
    lv_indev_set_long_press_time(lvTouchIndev, HOLD_PRESS_MS);      // hold 1s anywhere on the dashboard = Settings

    // Data install journal (Phone Update Bridge / online update): swap staged,
    // verified files in BEFORE the map data is loaded into PSRAM, then let the
    // installer roll back if the new set doesn't mount. See update/DataInstaller.h.
    bool dataApplied = installerBootApply();
    bool mountOk = sdMgrMount();
    installerAfterMount(mountOk); // rolls back + reboots if the NEW data can't mount
    if (dataApplied || installerRolledBackThisBoot()) showInstallResult(!dataApplied);

    // If the last shutdown was a "press Update data → reboot", handle the whole
    // download here, before any RAM-hungry task starts, then reboot back to
    // normal. dataUpdatePending() reads AND clears the flag, so a failed run
    // can't wedge us in a boot loop. This never returns when it fires.
    if (dataUpdatePending()) {
        runDataUpdateMode();
    }

    buildDashboard();
    buildSettingsScreen();
    showBootSplash(); // ~2s animated VRE logo, then cross-fades to the Dashboard (blocking)
#ifdef VIETHUD_P4
    // First boot on this panel / after a rotation change: calibrate touch
    // (gives up after 30 s untouched, keeping the nominal mapping).
    if (!touchCalValid()) touchCalibrationRun(true);
#endif
    // Welcome greeting AFTER the splash has finished (user-requested 2026-09-25):
    // the spoken "chào mừng" now plays once the dashboard is on screen, not over
    // the logo animation. SD is already mounted above, so the clip is available.
    audioQueueVoice("welcome/voice.mp3");

    sharedStateInit();
    gnssTaskStart();  // Core 0 — real GNSS M10N on UART2
#ifdef VIETHUD_P4
    phoneGpsBleStart(); // BLE (via the C6): phone GPS as NMEA over Nordic UART — net/PhoneGpsBle.h
#endif
    webPortalInit(); // Core 0 — WiFi AP + local web server, starts with WiFi OFF — see net/WebPortal.h
    // Just rebooted to install data pushed from a phone: bring the hotspot back
    // so the phone reconnects and its portal page can show the result.
    if (installerTakeWifiOnRequest()) webPortalRequestEnable(true);
    speedLimitManagerStart(); // Core 0 — microSD speed-limit/camera/sign map matching, see map/SpeedLimitManager.h
    tripLoggerStart(); // Core 0 — microSD trip/event CSV logging, see log/TripLogger.h
    // (Was disabled 2026-09-15 after two SPI-peripheral-contention
    // regressions — see pincfg.h's SD_MMC_CLK_PIN comment. Root cause: the
    // TF slot was never SPI at all, it's the ESP32-S3's dedicated SD_MMC
    // peripheral in 1-bit mode — confirmed from the vendor-adjacent demo
    // project's own working code. SdCardManager.cpp now uses SD_MMC.h,
    // which shares no hardware with the display's QSPI or the touch I2C
    // bus, so this is re-enabled.)
    refreshDashboard();
    lv_timer_create(simTimerCb, 150, NULL);
    lv_timer_create(burnInTimerCb, 60000, NULL);

    Serial.println("[viethud] running — hold the screen 1s to open Settings");
}

// Firmware OTA rollback (bootloader CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=1):
// tell the Arduino core NOT to auto-confirm a freshly OTA'd image at startup.
// loop() confirms it after kHealthyAfterMs of normal running; a crash/reset
// before that boots the previous firmware again. (C linkage: overrides the weak
// default in esp32-hal-misc.c.)
#ifdef VIETHUD_P4
// Flicker diagnostic (serial 'Z', 2026-09-29): cycles A/B/C twice, 15 s each,
// with the phase letter shown large in the top-right corner so the person
// looking at the panel can say which phases flicker:
//   A = normal rendering, B = presentation frozen on one frame (isolates the
//   render/swap pipeline), C = everything running except the map redraw
//   (isolates the map layer). v1 of C was "steady backlight"; the backlight
//   is now a steady HIGH at 100% anyway (DisplayDriverP4 backlightWrite).
extern volatile bool gMapPaused; // ui/Dashboard.cpp
static int sDiagPhase = -1;
static uint32_t sDiagT0 = 0, sDiagPhaseT0 = 0;
static lv_obj_t *sDiagLbl = nullptr;
static void flickerDiagStart() {
    sDiagT0 = millis();
    sDiagPhase = -1;
    if (!sDiagLbl) {
        sDiagLbl = lv_label_create(lv_layer_top());
        lv_obj_set_style_text_font(sDiagLbl, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(sDiagLbl, lv_color_hex(0xFFD400), 0);
        lv_obj_set_style_bg_color(sDiagLbl, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(sDiagLbl, LV_OPA_70, 0);
        lv_obj_set_style_pad_all(sDiagLbl, 6, 0);
        lv_obj_align(sDiagLbl, LV_ALIGN_TOP_RIGHT, -8, 36);
    }
    lv_obj_clear_flag(sDiagLbl, LV_OBJ_FLAG_HIDDEN);
    Serial.println("[diag] flicker test: A=normal B=frozen C=map paused, 15 s each, x2");
}
static void flickerDiagTick() {
    if (!sDiagLbl || lv_obj_has_flag(sDiagLbl, LV_OBJ_FLAG_HIDDEN)) return;
    uint32_t el = millis() - sDiagT0;
    int ph = (int)(el / 15000);
    if (ph >= 6) {
        displaySetFrozen(false);
        gMapPaused = false;
        lv_obj_add_flag(sDiagLbl, LV_OBJ_FLAG_HIDDEN);
        Serial.println("[diag] flicker test done");
        return;
    }
    if (ph != sDiagPhase) {
        sDiagPhase = ph;
        sDiagPhaseT0 = millis();
        static const char *kL[3] = {"A", "B", "C"};
        lv_label_set_text(sDiagLbl, kL[ph % 3]);
        displaySetFrozen(false);
        gMapPaused = (ph % 3 == 2);
        Serial.printf("[diag] phase %s\n", kL[ph % 3]);
    }
    // Phase B: let one frame with the "B" label reach the panel, then freeze.
    if (sDiagPhase % 3 == 1 && millis() - sDiagPhaseT0 > 400) displaySetFrozen(true);
}
#endif

extern "C" bool verifyRollbackLater() { return true; }
static const uint32_t kHealthyAfterMs = 30000;

void loop() {
    esp_task_wdt_reset();

    static bool sHealthyDone = false;
    if (!sHealthyDone && millis() > kHealthyAfterMs) {
        sHealthyDone = true;
        esp_ota_mark_app_valid_cancel_rollback(); // no-op unless this image is pending verification
        installerConfirmHealthy();                // new data survived 30 s -> drop its backups
    }

    static uint32_t lastTick = millis();
    static uint32_t lastHb = 0;
    uint32_t now0 = millis();
    // Kept temporarily: if a freeze recurs, the last [hb] line before the
    // next boot banner pinpoints how long loop() actually ran before it
    // stuck, without needing to reproduce with a debugger attached.
    if (now0 - lastHb > 1000) { lastHb = now0; Serial.printf("[hb %lu]\n", now0); }

    // Bench serial commands (see docs/VIETHUD_FW_2.2.0.md, "Lenh serial").
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == 'a') {
            audioSelfTest(); // play every chime + voice clip once, for speaker bench verification
        } else if (c == 'w') {
            bool on = !webPortalIsEnabled();
            webPortalRequestEnable(on); // bench: toggle WiFi/web without the touchscreen hold gesture
            Serial.printf("[debug] WiFi toggled via serial -> %s\n", on ? "ON" : "OFF");
        } else if (c == 's') {
            webPortalRequestEnable(true); // bench: scan nearby WiFi (needs radio on) and log results
            webPortalStartScan();
            Serial.println("[debug] WiFi scan requested via serial");
        } else if (c == 'T' || c == 'H' || c == 'G') {
            // Bench: synthetic touch. "T x y" tap, "H x y ms" hold, "G x1 y1 x2 y2 ms" drag.
            String line = Serial.readStringUntil('\n');
            int v[5] = {0, 0, 0, 0, 0};
            int n = sscanf(line.c_str(), "%d %d %d %d %d", &v[0], &v[1], &v[2], &v[3], &v[4]);
            int x0 = v[0], y0 = v[1], x1 = v[0], y1 = v[1], ms = 150;
            if (c == 'H' && n >= 3) ms = v[2];
            if (c == 'G' && n >= 5) { x1 = v[2]; y1 = v[3]; ms = v[4]; }
            sInjX0 = x0; sInjY0 = y0; sInjX1 = x1; sInjY1 = y1;
            sInjStartMs = millis();
            sInjEndMs = sInjStartMs + (ms > 0 ? ms : 1);
            Serial.printf("[touch] injected %c (%d,%d)->(%d,%d) %dms\n", c, x0, y0, x1, y1, ms);
        } else if (c == 'n') {
            // Bench: add a saved STA network without the touchscreen/portal —
            // "n<ssid>\t<password>\n". Used to put the device on a PC hotspot
            // so a PC can drive the Phone Update Bridge API end-to-end.
            String line = Serial.readStringUntil('\n');
            int tab = line.indexOf('\t');
            if (tab > 0) {
                String pass = line.substring(tab + 1);
                pass.trim();
                webPortalAddNetwork(line.substring(0, tab).c_str(), pass.c_str());
                Serial.printf("[debug] saved network via serial: \"%s\"\n", line.substring(0, tab).c_str());
            }
        } else if (c == 'S') {
            // Bench: simulated drive through real map data — "S lat lon heading kmh seconds".
            String line = Serial.readStringUntil('\n');
            float a = 0, b = 0, h = 0, v = 0, t = 0;
            if (sscanf(line.c_str(), "%f %f %f %f %f", &a, &b, &h, &v, &t) >= 5) gnssSimStart(a, b, h, v, t);
            else gnssSimStart(0, 0, 0, 0, 0);
        } else if (c == 'W') {
            // Bench: drive a polyline — "W kmh lat lon lat lon ..." (up to 48 points).
            String line = Serial.readStringUntil('\n');
            static float wl[48], wo[48];
            const char *q = line.c_str();
            char *end;
            float kmh = strtof(q, &end);
            int n = 0;
            while (n < 48) {
                q = end;
                float a = strtof(q, &end);
                if (end == q) break;
                q = end;
                float b = strtof(q, &end);
                if (end == q) break;
                wl[n] = a; wo[n] = b; n++;
            }
            if (n >= 2) gnssSimRoute(wl, wo, n, kmh);
        } else if (c == 'R') {
            // Road-following sim (gnss/RoadSim.h): "R lat lon [speedFactor]"
            // drives on real roads from there; a bare "R" stops it.
            String line = Serial.readStringUntil('\n');
            float a = 0, b = 0, f = 1.0f;
            int n = sscanf(line.c_str(), "%f %f %f", &a, &b, &f);
            if (n >= 2) roadSimRequestStart(a, b, n >= 3 ? f : 1.0f);
            else roadSimRequestStop();
        } else if (c == 'P') {
            // Bench: screenshot of the active screen as raw RGB565 over serial.
#ifdef VIETHUD_P4
            // DIRECT-mode frame is the whole current screen already (a 768KB
            // lv_snapshot_take would not fit the LVGL pool).
            {
                const uint16_t *fr = displayFrame();
                int fw = displayPhysWidth(), fh = displayPhysHeight();
                Serial.printf("[snap] %d %d %d\n", fw, fh, fw * 2);
                Serial.flush();
                Serial.write((const uint8_t *)fr, (size_t)fw * fh * 2);
                Serial.flush();
                Serial.println("\n[snap] end");
            }
            lv_draw_buf_t *snap = nullptr;
#else
            lv_draw_buf_t *snap = lv_snapshot_take(lv_screen_active(), LV_COLOR_FORMAT_RGB565);
#endif
            if (snap) {
                Serial.printf("[snap] %u %u %u\n", (unsigned)snap->header.w, (unsigned)snap->header.h,
                              (unsigned)snap->header.stride);
                Serial.flush();
                Serial.write(snap->data, (size_t)snap->header.stride * snap->header.h);
                Serial.flush();
                Serial.println("\n[snap] end");
                lv_draw_buf_destroy(snap);
            }
#ifndef VIETHUD_P4
            else {
                Serial.println("[snap] failed (out of memory)");
            }
#endif
        } else if (c == 'u') {
            // Bench trigger for the OTA data update. Uses the SAME reliable path
            // as the web + on-screen buttons: set the NVS flag and reboot into
            // "update mode", where the whole download runs before any RAM-hungry
            // task starts (the running app can't spare the ~16 KB contiguous the
            // TLS handshake needs — that was the "cannot fetch manifest" error).
            Serial.println("[debug] data update requested via serial -> scheduling reboot to update mode");
            dataUpdateSchedule(); // sets flag, then ESP.restart()
        }
#ifdef VIETHUD_P4
        else if (c == 'F') {
            displayTraceFrames(40);
        } else if (c == 'Z') {
            flickerDiagStart();
        } else if (c == 'K') {
            gTouchCalRequested = true; // run outside LVGL callbacks, below
        }
#endif
    }
#ifdef VIETHUD_P4
    flickerDiagTick();
    if (gTouchCalRequested) {
        gTouchCalRequested = false;
        touchCalibrationRun(false);
    }
#endif

    uint32_t now = millis();
    lv_tick_inc(now - lastTick);
    lastTick = now;
    lv_timer_handler();

    // Report where the frame time actually goes, so tuning stops being guesswork.
    if (now - lastStatsMs > 5000) {
        lastStatsMs = now;
        if (g_flushCount) {
            Serial.printf("[perf] frames=%lu  flush=%luus avg  canvas-copy=%luus avg\n", g_flushCount,
                          g_flushUs / g_flushCount, g_renderUs / g_flushCount);
        }
        g_flushUs = g_renderUs = g_flushCount = 0;
        // Live background map's own draw cost (ui/Dashboard.h) — separate
        // line since it only increments on the (usually rare) ticks that
        // actually redrew the map canvas, not every frame like the counters
        // above; see the plan's own note that the spec's "<=10ms" render
        // target needs measuring on real hardware, not assuming.
        if (g_mapDrawCount) {
            Serial.printf("[perf] map redraws=%lu  draw=%luus avg\n", g_mapDrawCount, g_mapDrawUs / g_mapDrawCount);
        }
        g_mapDrawUs = g_mapDrawCount = 0;
    }

    // Slow-leak early warning: if internal-RAM free space keeps trending
    // down run over run while its all-time minimum (tracked by the IDF
    // itself) keeps dropping too, something is leaking — worth catching
    // from a serial log over a multi-hour/day drive long before it actually
    // runs out and crashes. Internal RAM specifically (MALLOC_CAP_INTERNAL),
    // not the combined/default heap: LVGL objects and every FreeRTOS task
    // stack in this app come from internal RAM (see the draw-buffer
    // allocation above), while PSRAM (tracked separately below) sits mostly
    // idle and isn't the scarce resource here — mixing the two into one
    // "free heap" figure would silently include PSRAM's much larger, mostly
    // static headroom and mask a real internal-RAM leak trend. 30s cadence
    // is plenty for a slow trend; no need for anything finer-grained than a
    // human skimming the log.
    if (now - lastMemStatsMs > 30000) {
        lastMemStatsMs = now;
        Serial.printf("[mem] freeInternal=%uKB minFreeInternalEver=%uKB freePsram=%uKB\n",
                      (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
                      (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
                      (unsigned)(ESP.getFreePsram() / 1024));
        // LVGL's own internal pool (LV_MEM_SIZE, lv_conf.h) is a separate
        // budget from the general internal-RAM heap tracked just above —
        // kept as a permanent low-cost telemetry line (added 2026-09-16
        // RAM audit) since that pool's own real max_used is what
        // LV_MEM_SIZE should be sized against, not a guess; see lv_conf.h's
        // own comment for the measurement this constant was picked from.
        lv_mem_monitor_t mon;
        lv_mem_monitor(&mon);
        Serial.printf("[mem] lvgl pool total=%uB maxUsed=%uB (%u%%) freeBiggest=%uB fragPct=%u%%\n",
                      (unsigned)mon.total_size, (unsigned)mon.max_used, mon.used_pct,
                      (unsigned)mon.free_biggest_size, mon.frag_pct);
        // This loop() IS the Arduino framework's own loopTask (Core 1), not
        // one this project creates itself, but its stack still comes out of
        // internal RAM like every other task's, so it's tracked here too.
        Serial.printf("[mem] loopTask stack high-water mark: %u bytes free\n",
                      (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)));
        // speedLimitTask (Core 0) runs the map matcher AND the live map render +
        // SD tile I/O — the heaviest, deepest call chain in the build and the
        // one that has twice tripped the FreeRTOS stack canary. Watch this while
        // driving in a dense area: if it approaches 0, the in-city crash is a
        // stack overflow here (stack raised to 16384 on 2026-09-25).
        Serial.printf("[mem] speedLimitTask stack high-water mark: %u bytes free\n",
                      (unsigned)speedLimitTaskStackFreeBytes());
    }
    delay(1);
}

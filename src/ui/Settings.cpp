#include "Settings.h"
#include "Dashboard.h" // dashboardScreen (Back button), applyConfig()
#include "display/DisplayDriver.h" // gfx->width()/height() — orientation-aware layout
#include "core/AppConfig.h"
#include "core/NvsStore.h"
#include "core/SharedState.h" // gnssSnapshot()/roadInfoSnapshot() for the Sensors diagnostics panel
#include "map/SpeedLimitManager.h" // speedSourceStr() — Speed Map group in the Sensors tab
#include "net/WebPortal.h"    // webPortalIsEnabled()/webPortalRequestEnable() — WiFi tab
#include "net/DataUpdater.h"  // dataUpdateStart()/GetStatus() — WiFi tab "Update data" button
#include "net/UpdateApi.h"    // updateApiBusy() — a phone is pushing data right now
#include "update/DataInstaller.h" // installerProgress() — phone->device transfer progress
#include "touch/TouchTask.h"      // gTouchCalRequested (P4 touch calibration button)
#include <string.h>

// ---------------------------------------------------------------------
// Settings screen (spec sections 16-17, 55-61)
// ---------------------------------------------------------------------
struct SliderBinding {
    lv_obj_t *slider;
    lv_obj_t *valLabel;
    float *target;
    float divisor;
    const char *unit;
};
// 7 in use (Brightness, Dim-after-stopped, GNSS speed-filter smoothing, GNSS
// fix timeout, GPS speed calibration, speed-limit-ahead warn distance,
// camera warn distance — the last 3 added 2026-09-22) after radar removal
// 2026-09-21 dropped the ~19 radar-only slider rows this array used to size
// against (was 32 wide for that reason — see git history). Checked by
// counting addSliderRow() call sites directly rather than trusting an old
// comment.
static SliderBinding sliderBindings[12]; // bumped from 8 (2026-09-24): 9 slider rows now (added Overspeed offset + Default limit)
static int sliderCount = 0;

struct SwitchBinding {
    lv_obj_t *sw;
    bool *target;
};
// 2 in use (Alert audio enabled, Trip logging) after radar removal
// 2026-09-21 dropped the radar-only demo-mode/mount-flip switches this
// array used to size against (was 12 wide for that reason).
static SwitchBinding switchBindings[24];
static int switchCount = 0;

static lv_obj_t *settingsStatusLabel;
static lv_obj_t *restartConfirmOverlay = nullptr;

// WiFi tab widgets — declared up top since onWifiSwitchChanged()/
// onWifiKbReadyOrCancel() below (both fairly early in the file) reference
// them; built in buildSettingsScreen() near the bottom, same
// declare-early/build-late split refreshSensorsPanel()'s own widget
// pointers already use.
static lv_obj_t *wifiEnableSwitch, *wifiSsidTa, *wifiPasswordTa;
static lv_obj_t *staSsidTa = nullptr, *staPasswordTa = nullptr; // legacy manual STA fields (removed from UI; kept as guarded nullptrs)
static lv_obj_t *staStatusVal = nullptr;                        // STA connection info row
static lv_obj_t *staSavedVal = nullptr;                         // saved-networks summary row (WiFi Manager)
// "WiFi setup" overlay: scan nearby networks + pick one + enter password, on a
// non-scrolling full-screen modal so the on-screen keyboard works reliably
// (2026-09-25 — user couldn't type a password on the scrollable WiFi tab).
static const int kCategoryWifi = 3; // index of the WiFi tab (kCategoryNames order)
static int g_activeCategory = 0;    // which tab is currently shown (idle-return exemption)
// "WiFi setup" overlay (2026-09-26): NO on-device password typing. It shows a QR
// code that the phone's camera scans to join the device's own AP; the phone then
// opens the web portal (captive portal auto-pops) and configures WiFi there, with
// the phone's real keyboard. When the user saves a network on the phone, the WiFi
// Manager auto-connects the device to it and auto-checks for a data update — the
// overlay's status line reflects that live.
static lv_obj_t *wifiScanOverlay = nullptr;   // the QR setup overlay (name kept for wiring)
static lv_obj_t *wifiQrObj = nullptr;         // lv_qrcode
static lv_obj_t *scanConnLbl = nullptr;       // live connection status inside the overlay
static lv_obj_t *scanSavedLbl = nullptr;      // "saved networks" summary inside the overlay
static char g_qrPayload[128] = "";            // last-built WIFI: QR string (rebuild only on change)

// Formats a settings value: no decimal for whole numbers ("100 m", "3 min"),
// one decimal otherwise ("0.3", "1.5"). Cleaner than the old always-"%.1f"
// which showed "100.0 m" / "3.0 min" (2026-09-24 UI polish for glanceability).
static void fmtSettingVal(char *buf, size_t n, float v, const char *unit) {
    if (v == (float)(long)v) snprintf(buf, n, "%ld%s", (long)v, unit ? unit : "");
    else snprintf(buf, n, "%.1f%s", (double)v, unit ? unit : "");
}


static void onSliderChanged(lv_event_t *e) {
    SliderBinding *b = (SliderBinding *)lv_event_get_user_data(e);
    int32_t raw = lv_slider_get_value(b->slider);
    *(b->target) = raw / b->divisor;
    char buf[24];
    fmtSettingVal(buf, sizeof(buf), *(b->target), b->unit);
    lv_label_set_text(b->valLabel, buf);
    lv_label_set_text(settingsStatusLabel, "");
    clampConfig(cfg);
    // applyConfig() re-writes the backlight PWM duty — only the brightness
    // slider needs that, but this used to fire on EVERY slider's every
    // drag tick (an LVGL slider fires VALUE_CHANGED many times per drag),
    // so dragging e.g. "Max range" was rewriting the backlight duty
    // dozens of times for no reason. User-reported 2026-09-15 as Settings
    // feeling laggy.
    if (b->target == &cfg.brightness || b->target == &cfg.audioVolume) applyConfig(); // both are pushed to hardware in applyConfig()
    // Rotation can't apply live (see AppConfig.h's screenRotation comment —
    // both screens are laid out once at boot for whichever orientation was
    // active then) — say so immediately rather than let the slider silently
    // do nothing, which is what every OTHER slider here does instead.
    if (b->target == &cfg.brightnessMode) applyConfig(); // Auto/Manual backlight applies live
    if (b->target == &cfg.screenRotation) {
        saveConfigToNVS(cfg);
        lv_label_set_text(settingsStatusLabel, "Rotation saved. Restart to apply.");
        if (restartConfirmOverlay) {
            lv_obj_clear_flag(restartConfirmOverlay, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void onSwitchChanged(lv_event_t *e) {
    SwitchBinding *b = (SwitchBinding *)lv_event_get_user_data(e);
    *(b->target) = lv_obj_has_state(b->sw, LV_STATE_CHECKED);
    clampConfig(cfg); // no switch affects brightness, so no applyConfig() call needed here
    // Persist immediately so toggles (esp. the map mode: JPEG/vector/heading-up)
    // survive a reboot without needing the footer Save — user-requested
    // 2026-09-24 ("ghi nhớ lưu chọn bản đồ ... sau khi khởi động"). Switches are
    // discrete/infrequent, so a NVS write per toggle is fine (unlike sliders).
    saveConfigToNVS(cfg);
    lv_label_set_text(settingsStatusLabel, "Saved");
}

// Not wired through the generic SwitchBinding/onSwitchChanged above:
// WiFi's on/off state isn't an AppConfig/NVS field (see AppConfig.h's
// wifiSsid/wifiPassword comment — only credentials persist, never on/off),
// so there's no `bool *target` in cfg for the generic binding to point at.
// This talks to net/WebPortal.h directly instead, same call the Dashboard's
// WiFi hold gesture (4s tier) makes.
static void onWifiSwitchChanged(lv_event_t *e) {
    bool on = lv_obj_has_state(wifiEnableSwitch, LV_STATE_CHECKED);
    webPortalRequestEnable(on);
    Serial.printf("[uidemo] WiFi %s via Settings switch\n", on ? "ON" : "OFF");
}

// On-screen keyboard for wifiSsidTa/wifiPasswordTa — this screen's first use
// of lv_textarea/lv_keyboard (every other input here is a slider/switch, no
// text entry has ever been needed before). Bound to LV_EVENT_CLICKED rather
// than LV_EVENT_FOCUSED: LVGL's focus/group model is built around encoder-
// style indevs, and this app only ever has a pointer/touch indev with no
// lv_group in use elsewhere — CLICKED is unambiguous regardless of that and
// is exactly the gesture a touchscreen keyboard should respond to anyway.
static lv_obj_t *wifiKeyboard;

static void onWifiTaClicked(lv_event_t *e) {
    lv_obj_t *ta = lv_event_get_target_obj(e);
    lv_keyboard_set_textarea(wifiKeyboard, ta);
    lv_obj_clear_flag(wifiKeyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(wifiKeyboard);
}

// Commits whichever textarea the keyboard is currently bound to into cfg —
// called on the keyboard's own Ready/Cancel (both close it; Cancel doesn't
// revert here since there's no separate "draft" copy to revert TO, same
// as every slider/switch in this screen applying live and only NVS-
// persisting on the footer's Save button).
static void onWifiKbReadyOrCancel(lv_event_t *) {
    // Only the device's own AP (hotspot) SSID/password are typed on-device now —
    // the network to CONNECT TO is configured from the phone via the QR/web flow,
    // so there are no STA text fields here anymore.
    lv_obj_t *ta = lv_keyboard_get_textarea(wifiKeyboard);
    if (ta == wifiSsidTa) {
        strncpy(cfg.wifiSsid, lv_textarea_get_text(ta), sizeof(cfg.wifiSsid) - 1);
        cfg.wifiSsid[sizeof(cfg.wifiSsid) - 1] = '\0';
    } else if (ta == wifiPasswordTa) {
        strncpy(cfg.wifiPassword, lv_textarea_get_text(ta), sizeof(cfg.wifiPassword) - 1);
        cfg.wifiPassword[sizeof(cfg.wifiPassword) - 1] = '\0';
    }
    lv_keyboard_set_textarea(wifiKeyboard, NULL);
    lv_obj_add_flag(wifiKeyboard, LV_OBJ_FLAG_HIDDEN);
}

// ---- "WiFi setup" overlay: QR to join the device AP, configure on the phone ----

// (Re)build the WIFI: QR payload from the current AP credentials. Only rebuilds
// the QR bitmap when the string actually changed (called on the refresh timer).
static void wifiQrRebuild() {
    if (!wifiQrObj) return;
    char ap[40];
    webPortalApSsid(ap, sizeof(ap));
    // Minimal escaping of the WIFI: reserved chars (\ ; , : ").
    auto esc = [](const char *in, char *out, size_t cap) {
        size_t o = 0;
        for (const char *p = in; *p && o < cap - 2; p++) {
            if (*p == '\\' || *p == ';' || *p == ',' || *p == ':' || *p == '"') out[o++] = '\\';
            out[o++] = *p;
        }
        out[o] = '\0';
    };
    char es[48], ep[80];
    esc(ap, es, sizeof(es));
    bool secured = strlen(cfg.wifiPassword) >= 8; // WPA2 min; else the AP is open
    char payload[128];
    if (secured) {
        esc(cfg.wifiPassword, ep, sizeof(ep));
        snprintf(payload, sizeof(payload), "WIFI:T:WPA;S:%s;P:%s;;", es, ep);
    } else {
        snprintf(payload, sizeof(payload), "WIFI:T:nopass;S:%s;;", es);
    }
    if (strcmp(payload, g_qrPayload) != 0) {
        strncpy(g_qrPayload, payload, sizeof(g_qrPayload) - 1);
        g_qrPayload[sizeof(g_qrPayload) - 1] = '\0';
        lv_qrcode_update(wifiQrObj, payload, strlen(payload));
    }
}

// Phone-joined detection for the QR screen: g_qrPrevClients = AP client count at
// the previous refresh (-1 = take a fresh baseline), g_qrJoinedAtMs = when a
// phone joined while the screen was up (0 = none pending).
static int g_qrPrevClients = -1;
static lv_obj_t *qrWifiSwitch = nullptr;  // WiFi on/off switch on the QR screen
static uint32_t g_qrJoinedAtMs = 0;
static lv_obj_t *connToast = nullptr;   // "phone connected" pill on lv_layer_top()
static uint32_t connToastUntilMs = 0;

static void showPhoneConnectedToast() {
    if (!connToast) {
        connToast = lv_label_create(lv_layer_top());
        lv_obj_set_style_bg_color(connToast, lv_color_hex(0x10261A), 0);
        lv_obj_set_style_bg_opa(connToast, LV_OPA_90, 0);
        lv_obj_set_style_border_color(connToast, lv_color_hex(0x3CC46E), 0);
        lv_obj_set_style_border_width(connToast, 1, 0);
        lv_obj_set_style_radius(connToast, 14, 0);
        lv_obj_set_style_pad_hor(connToast, 14, 0);
        lv_obj_set_style_pad_ver(connToast, 6, 0);
        lv_obj_set_style_text_color(connToast, lv_color_hex(0xDFF5E6), 0);
        lv_label_set_text(connToast, LV_SYMBOL_OK " Phone connected · settings page: 192.168.4.1");
        lv_obj_align(connToast, LV_ALIGN_TOP_MID, 0, 40);
    }
    lv_obj_clear_flag(connToast, LV_OBJ_FLAG_HIDDEN);
    connToastUntilMs = millis() + 6000;
}

static void onWifiScanOpen(lv_event_t *) {
    // Opening the screen does NOT change WiFi (2026-09-26, user request): the
    // switch shows the current state; the user flips it to turn WiFi on/off.
    g_qrPayload[0] = '\0';          // force a rebuild for the (now-active) AP creds
    g_qrPrevClients = -1;           // baseline taken on the first refresh (see refreshScanListIfOpen)
    g_qrJoinedAtMs = 0;
    if (qrWifiSwitch) {
        if (webPortalRequestedOn()) lv_obj_add_state(qrWifiSwitch, LV_STATE_CHECKED);
        else lv_obj_remove_state(qrWifiSwitch, LV_STATE_CHECKED);
    }
    wifiQrRebuild();
    lv_obj_clear_flag(wifiScanOverlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(wifiScanOverlay);
}
static void onWifiScanClose(lv_event_t *) {
    Serial.println("[ui] WiFi QR screen closed");
    // WiFi stays exactly as the switch left it (opening no longer turns it on,
    // so closing no longer turns it off); the 10 min auto-off still applies.
    lv_obj_add_flag(wifiScanOverlay, LV_OBJ_FLAG_HIDDEN);
    g_activeCategory = 0; // leave the WiFi context so the idle-return timer re-arms
    // The WiFi "menu" IS this QR screen now, so closing it returns straight to the
    // Dashboard rather than to an (intentionally empty) WiFi tab behind it.
    lv_screen_load(dashboardScreen);
}

// Layout for the 480x320 landscape panel — ONE QR (join the device WiFi; the
// phone then opens the portal by itself / at 192.168.4.1):
//   [        ][ WiFi name / password / portal address ]
//   [ QR     ][ status (phone connected / receiving %) ]
//   [        ][ progress bar                           ]
//   [caption ][                               [ Đóng ] ]
static lv_obj_t *bridgeBar = nullptr;         // phone -> device transfer progress
static lv_obj_t *bridgeToast = nullptr;       // small pill on lv_layer_top() while a phone is updating

static lv_obj_t *makeQr(lv_obj_t *parent, int size, int x, int y) {
    lv_obj_t *q = lv_qrcode_create(parent);
    lv_qrcode_set_size(q, size);
    lv_qrcode_set_dark_color(q, lv_color_black());
    lv_qrcode_set_light_color(q, lv_color_white());
    lv_obj_set_style_border_color(q, lv_color_white(), 0);
    lv_obj_set_style_border_width(q, 6, 0); // quiet zone so phone cameras lock on
    lv_obj_set_pos(q, x, y);
    return q;
}

static void buildWifiScanOverlay(lv_obj_t *parent) {
    int W = gfx->width(), Hh = gfx->height();
    wifiScanOverlay = lv_obj_create(parent);
    lv_obj_set_pos(wifiScanOverlay, 0, 0);
    lv_obj_set_size(wifiScanOverlay, W, Hh);
    lv_obj_set_style_bg_color(wifiScanOverlay, lv_color_hex(0x0B0F14), 0);
    lv_obj_set_style_bg_opa(wifiScanOverlay, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(wifiScanOverlay, 0, 0);
    lv_obj_set_style_radius(wifiScanOverlay, 0, 0);
    lv_obj_set_style_pad_all(wifiScanOverlay, 0, 0);
    lv_obj_clear_flag(wifiScanOverlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wifiScanOverlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *title = lv_label_create(wifiScanOverlay);
    lv_label_set_text(title, LV_SYMBOL_WIFI "  Connect phone");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_pos(title, 12, 8);

    // The one QR: joins the phone to the device hotspot (WIFI: payload).
    int qy = 34;
    int qsz = Hh - qy - 50; // leave room for the caption underneath
    if (qsz > 220) qsz = 220;
    if (qsz < 120) qsz = 120;
    wifiQrObj = makeQr(wifiScanOverlay, qsz, 14, qy);
    lv_qrcode_update(wifiQrObj, "WIFI:;", 6); // placeholder until wifiQrRebuild()
    lv_obj_t *cap = lv_label_create(wifiScanOverlay);
    lv_obj_set_width(cap, qsz + 12);
    lv_obj_set_style_text_align(cap, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(cap, lv_color_hex(0xCCD6E0), 0);
    lv_label_set_text(cap, "Scan with the camera to join");
    lv_obj_set_pos(cap, 14, qy + qsz + 16);

    // Right column: the same credentials in text (manual join), then live status,
    // the transfer progress bar and Close at the bottom.
    int rx = 14 + qsz + 12 + 20;
    int rw = W - rx - 12;
    lv_obj_t *swLbl = lv_label_create(wifiScanOverlay);
    lv_label_set_text(swLbl, LV_SYMBOL_WIFI "  Wi-Fi");
    lv_obj_set_style_text_color(swLbl, lv_color_white(), 0);
    lv_obj_set_pos(swLbl, rx, qy + 8);
    qrWifiSwitch = lv_switch_create(wifiScanOverlay);
    lv_obj_set_size(qrWifiSwitch, 64, 32);
    lv_obj_set_pos(qrWifiSwitch, W - 12 - 64, qy + 2);
    lv_obj_set_ext_click_area(qrWifiSwitch, 8);
    lv_obj_set_style_bg_color(qrWifiSwitch, lv_color_hex(0x34C46A), LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_add_event_cb(
        qrWifiSwitch, [](lv_event_t *e) {
            bool on = lv_obj_has_state((lv_obj_t *)lv_event_get_target(e), LV_STATE_CHECKED);
            g_qrPrevClients = -1; // fresh baseline for the "phone joined" detection
            webPortalRequestEnable(on);
            Serial.printf("[ui] WiFi switched %s from the WiFi screen\n", on ? "ON" : "OFF");
        },
        LV_EVENT_VALUE_CHANGED, NULL);

    scanSavedLbl = lv_label_create(wifiScanOverlay);
    lv_label_set_long_mode(scanSavedLbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(scanSavedLbl, rw);
    lv_obj_set_style_text_color(scanSavedLbl, lv_color_hex(0xCCD6E0), 0);
    lv_obj_set_style_text_line_space(scanSavedLbl, 3, 0);
    lv_obj_set_pos(scanSavedLbl, rx, qy + 46);

    int btnW = rw;
    scanConnLbl = lv_label_create(wifiScanOverlay);
    lv_label_set_long_mode(scanConnLbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(scanConnLbl, rw);
    lv_obj_set_style_text_color(scanConnLbl, lv_color_hex(0x8FA0B4), 0);
    lv_obj_set_pos(scanConnLbl, rx, qy + 134);

    bridgeBar = lv_bar_create(wifiScanOverlay);
    lv_obj_set_size(bridgeBar, rw, 10);
    lv_obj_set_pos(bridgeBar, rx, Hh - 72);
    lv_obj_set_style_bg_color(bridgeBar, lv_color_hex(0x1E2A38), LV_PART_MAIN);
    lv_obj_set_style_bg_color(bridgeBar, lv_color_hex(0x3DA5FF), LV_PART_INDICATOR);
    lv_bar_set_range(bridgeBar, 0, 100);
    lv_obj_add_flag(bridgeBar, LV_OBJ_FLAG_HIDDEN);

    // Close = a strip running to the bottom AND right screen edges. Real taps
    // in this corner were logged at y=319 (the panel reports touches near the
    // bottom edge lower than the finger), i.e. BELOW the old button (y 270-310),
    // so "Đóng" never fired. Reaching the edges + an extended click area makes
    // any such clamped touch land on it.
    (void)btnW;
    lv_obj_t *closeBtn = lv_button_create(wifiScanOverlay);
    lv_obj_set_size(closeBtn, W - rx + 12, 58);
    lv_obj_set_pos(closeBtn, rx - 12, Hh - 58);
    lv_obj_set_ext_click_area(closeBtn, 10);
    lv_obj_set_style_bg_color(closeBtn, lv_color_hex(0x2A3644), 0);
    lv_obj_set_style_radius(closeBtn, 0, 0);
    lv_obj_set_style_shadow_width(closeBtn, 0, 0);
    lv_obj_add_event_cb(closeBtn, onWifiScanClose, LV_EVENT_CLICKED, NULL);
    lv_obj_t *closeLbl = lv_label_create(closeBtn);
    lv_label_set_text(closeLbl, "Close");
    lv_obj_center(closeLbl);
}

// Human text for the phone-update (bridge) state; returns true while a phone
// update is in progress so the caller can show the progress bar.
static bool bridgeStatusText(char *buf, size_t cap, int *pct) {
    InstallerProgress p = installerProgress();
    bool active = updateApiBusy() && (p.state == INST_RECEIVING || p.state == INST_READY ||
                                      p.state == INST_VERIFYING || p.state == INST_COMMITTED);
    *pct = p.bytesTotal ? (int)((uint64_t)p.bytesDone * 100 / p.bytesTotal) : 0;
    if (!active) return false;
    switch (p.state) {
    case INST_VERIFYING: snprintf(buf, cap, LV_SYMBOL_REFRESH " Checking data..."); break;
    case INST_COMMITTED: snprintf(buf, cap, LV_SYMBOL_OK " Data OK — restart to install"); break;
    default:
        snprintf(buf, cap, LV_SYMBOL_DOWNLOAD " Receiving data from phone  %d%%\n%u / %u KB · file %u/%u", *pct,
                 (unsigned)(p.bytesDone / 1024), (unsigned)(p.bytesTotal / 1024),
                 (unsigned)(p.filesDone < p.filesTotal ? p.filesDone + 1 : p.filesTotal), (unsigned)p.filesTotal);
    }
    return true;
}

// Small pill on the top layer so the driver sees a phone update progressing
// even with this overlay closed (Dashboard showing). Only text changes -> only
// the pill's own area is redrawn.
static void refreshBridgeToast() {
    bool overlayOpen = wifiScanOverlay && !lv_obj_has_flag(wifiScanOverlay, LV_OBJ_FLAG_HIDDEN);
    char b[120];
    int pct = 0;
    bool active = bridgeStatusText(b, sizeof(b), &pct) && !overlayOpen;
    if (!active) {
        if (bridgeToast && !lv_obj_has_flag(bridgeToast, LV_OBJ_FLAG_HIDDEN)) lv_obj_add_flag(bridgeToast, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    if (!bridgeToast) {
        bridgeToast = lv_label_create(lv_layer_top());
        lv_obj_set_style_bg_color(bridgeToast, lv_color_hex(0x0F2238), 0);
        lv_obj_set_style_bg_opa(bridgeToast, LV_OPA_90, 0);
        lv_obj_set_style_border_color(bridgeToast, lv_color_hex(0x3DA5FF), 0);
        lv_obj_set_style_border_width(bridgeToast, 1, 0);
        lv_obj_set_style_radius(bridgeToast, 14, 0);
        lv_obj_set_style_pad_hor(bridgeToast, 12, 0);
        lv_obj_set_style_pad_ver(bridgeToast, 5, 0);
        lv_obj_set_style_text_color(bridgeToast, lv_color_hex(0xDCEBFA), 0);
        lv_obj_align(bridgeToast, LV_ALIGN_BOTTOM_MID, 0, -6);
    }
    InstallerProgress p = installerProgress();
    char t[64];
    if (p.state == INST_VERIFYING) snprintf(t, sizeof(t), LV_SYMBOL_REFRESH " Checking data...");
    else if (p.state == INST_COMMITTED) snprintf(t, sizeof(t), LV_SYMBOL_OK " Restarting soon to install data");
    else snprintf(t, sizeof(t), LV_SYMBOL_DOWNLOAD " Receiving data from phone %d%%", pct);
    if (strcmp(lv_label_get_text(bridgeToast), t) != 0) lv_label_set_text(bridgeToast, t);
    if (lv_obj_has_flag(bridgeToast, LV_OBJ_FLAG_HIDDEN)) lv_obj_clear_flag(bridgeToast, LV_OBJ_FLAG_HIDDEN);
}

// Called from refreshSensorsPanel while the overlay is open: keep the QR current
// (AP creds can change) and show the device hotspot creds + live status.
static void refreshScanListIfOpen() {
    if (!wifiScanOverlay || lv_obj_has_flag(wifiScanOverlay, LV_OBJ_FLAG_HIDDEN)) return;

    // A phone just joined the hotspot while this screen is up: let the green
    // "Điện thoại đã kết nối" line show for ~1.5 s, then go back to the Dashboard
    // (the phone opens the portal by itself). A phone that was ALREADY connected
    // when the screen was opened doesn't trigger this (the QR stays up if reopened).
    int clientsNow = webPortalClientCount();
    if (g_qrPrevClients < 0) g_qrPrevClients = clientsNow;
    if (g_qrPrevClients == 0 && clientsNow > 0) g_qrJoinedAtMs = millis() | 1;
    if (clientsNow == 0) g_qrJoinedAtMs = 0;
    g_qrPrevClients = clientsNow;
    if (g_qrJoinedAtMs && millis() - g_qrJoinedAtMs >= 1500 && !updateApiBusy()) {
        g_qrJoinedAtMs = 0;
        Serial.printf("[ui] phone joined the hotspot (%d client(s)) -> back to Dashboard\n", clientsNow);
        showPhoneConnectedToast();
        onWifiScanClose(nullptr);
        return;
    }
    wifiQrRebuild();

    // Switch mirrors the requested state (auto-off / 4 s hold can change it too);
    // WiFi off -> the QR is dimmed and the status says so.
    bool wifiWanted = webPortalRequestedOn();
    if (qrWifiSwitch && lv_obj_has_state(qrWifiSwitch, LV_STATE_CHECKED) != wifiWanted) {
        if (wifiWanted) lv_obj_add_state(qrWifiSwitch, LV_STATE_CHECKED);
        else lv_obj_remove_state(qrWifiSwitch, LV_STATE_CHECKED);
    }
    if (wifiQrObj) {
        lv_opa_t want = wifiWanted ? LV_OPA_COVER : LV_OPA_20;
        if (lv_obj_get_style_opa(wifiQrObj, 0) != want) lv_obj_set_style_opa(wifiQrObj, want, 0);
    }

    if (scanSavedLbl) {
        char ap[40];
        webPortalApSsid(ap, sizeof(ap));
        bool secured = strlen(cfg.wifiPassword) >= 8;
        char buf[240];
        int n = snprintf(buf, sizeof(buf), "Name: %s\nPassword: %s\nPage: 192.168.4.1",
                         ap, secured ? cfg.wifiPassword : "(none)");
        // Also on a WiFi network (home / phone hotspot): the portal is reachable
        // at this address from any device on that same network.
        char staIp[24];
        if (webPortalStaIp(staIp, sizeof(staIp)) && n > 0 && n < (int)sizeof(buf))
            snprintf(buf + n, sizeof(buf) - n, "\nNetwork \"%s\": %s", cfg.staSsid, staIp);
        if (strcmp(lv_label_get_text(scanSavedLbl), buf) != 0) lv_label_set_text(scanSavedLbl, buf);
    }

    if (scanConnLbl) {
        char buf[200];
        int pct = 0;
        bool bridging = bridgeStatusText(buf, sizeof(buf), &pct);
        uint32_t color = 0x3DA5FF;
        if (!bridging && !wifiWanted) {
            snprintf(buf, sizeof(buf), "Wi-Fi is off.\nTurn the switch on to connect a phone.");
            color = 0xE5B53A;
        } else if (!bridging) {
            int clients = webPortalClientCount();
            char ip[24];
            char inet[80] = "";
            if (webPortalStaIp(ip, sizeof(ip))) {
                if (dataUpdateAvailable())
                    snprintf(inet, sizeof(inet), "\nNew data %s available — open the page to update", dataUpdateRemoteVersion());
            }
            if (clients > 0) {
                snprintf(buf, sizeof(buf), LV_SYMBOL_OK " Phone connected (%d)\nThe settings page opens on the phone\n(or go to 192.168.4.1)%s",
                         clients, inet);
                color = 0x34C46A;
            } else {
                snprintf(buf, sizeof(buf), "Waiting for a phone...\nNo home Wi-Fi needed to update data.%s", inet);
                color = 0x8FA0B4;
            }
        }
        lv_obj_set_style_text_color(scanConnLbl, lv_color_hex(color), 0);
        if (strcmp(lv_label_get_text(scanConnLbl), buf) != 0) lv_label_set_text(scanConnLbl, buf);
        if (bridgeBar) {
            if (bridging) {
                lv_obj_clear_flag(bridgeBar, LV_OBJ_FLAG_HIDDEN);
                lv_bar_set_value(bridgeBar, pct, LV_ANIM_OFF);
            } else if (!lv_obj_has_flag(bridgeBar, LV_OBJ_FLAG_HIDDEN)) {
                lv_obj_add_flag(bridgeBar, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

// Row layout tuned for the ~372px-wide / 260px-tall LANDSCAPE category
// content panel (see buildSettingsScreen) — panels don't scroll (except the
// Sensors tab), so every category's total row count x pitch must stay under
// 260px there. Tightened from 30/24px to 26/22px 2026-09-14 (originally for
// the now-removed Radar tab); re-check this budget before adding more rows
// to any tab.
//
// Width-aware (user-requested 2026-09-15, screen rotation): portrait's
// content panel is ~220px wide instead of landscape's ~380px — too narrow
// for name+slider+value on one line at the fixed x=148/x=306 offsets below,
// so under a threshold this stacks the slider+value onto their own line
// under the name instead of computing fractional widths from an arbitrary
// panel size. Landscape's own numbers are UNCHANGED (same offsets as
// before) specifically so the already-verified landscape layout stays
// pixel-identical — this is an ADDED case, not a generalization that also
// touches the existing one.
static const int kNarrowPanelThreshold = 300;

static void addSliderRow(lv_obj_t *parent, int &y, const char *name, float *target, int32_t minRaw, int32_t maxRaw,
                          float divisor, const char *unit) {
    bool narrow = lv_obj_get_width(parent) < kNarrowPanelThreshold;

    lv_obj_t *nameLbl = lv_label_create(parent);
    lv_label_set_text(nameLbl, name);
    lv_obj_set_style_text_color(nameLbl, lv_color_hex(0xCCD6E0), 0);

    lv_obj_t *slider = lv_slider_create(parent);
    lv_slider_set_range(slider, minRaw, maxRaw);
    lv_slider_set_value(slider, (int32_t)((*target) * divisor), LV_ANIM_OFF);

    lv_obj_t *valLbl = lv_label_create(parent);
    lv_obj_set_style_text_color(valLbl, lv_color_white(), 0);

    int rowH;
    if (narrow) {
        lv_obj_set_pos(nameLbl, 4, y);
        int sliderW = lv_obj_get_width(parent) - 4 - 54 - 6; // left margin, value column, gap
        if (sliderW < 60) sliderW = 60; // floor for an extreme-case panel — stays usable, may just crowd the value column
        lv_obj_set_size(slider, sliderW, 10);
        lv_obj_set_pos(slider, 4, y + 19);
        lv_obj_set_pos(valLbl, lv_obj_get_width(parent) - 54, y + 16);
        rowH = 38;
    } else {
        lv_obj_set_size(slider, 150, 10);
        lv_obj_set_pos(nameLbl, 4, y + 5);
        lv_obj_set_pos(slider, 148, y + 8);
        lv_obj_set_pos(valLbl, 306, y + 5);
        rowH = 26;
    }

    SliderBinding *b = &sliderBindings[sliderCount++];
    b->slider = slider;
    b->valLabel = valLbl;
    b->target = target;
    b->divisor = divisor;
    b->unit = unit;
    lv_obj_add_event_cb(slider, onSliderChanged, LV_EVENT_VALUE_CHANGED, b);

    char buf[24];
    fmtSettingVal(buf, sizeof(buf), *target, unit);
    lv_label_set_text(valLbl, buf);

    y += rowH;
}

// Small fixed set of named choices (Theme: Auto/Light/Dark, Rotation:
// 0/90/180/270) — a slider hides which discrete value is active and makes
// picking a specific one fiddly (user-reported 2026-09-16: "nut gat chua
// hop ly", asked for tick/choice buttons per option instead). Renders one
// row of `count` checkable buttons, all mutually exclusive (manual
// check/uncheck below — LVGL has no built-in radio-group for plain buttons).
struct ChoiceBinding {
    lv_obj_t *btns[4]; // 4 covers every current use (Theme=3, Rotation=4, Direction=3)
    int count;
    float *target;
};
static ChoiceBinding choiceBindings[8]; // 3 in use as of Direction (2026-09-21) — 1 slot free
static int choiceCount = 0;

static void onChoiceBtnClicked(lv_event_t *e) {
    lv_obj_t *clicked = lv_event_get_target_obj(e);
    ChoiceBinding *b = (ChoiceBinding *)lv_event_get_user_data(e);
    for (int i = 0; i < b->count; i++) {
        if (b->btns[i] == clicked) {
            *b->target = (float)i;
            lv_obj_add_state(b->btns[i], LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(b->btns[i], LV_STATE_CHECKED);
        }
    }
    clampConfig(cfg);
    lv_label_set_text(settingsStatusLabel, "");
    Serial.printf("[ui] choice %.0f selected\n", (double)*b->target);
    // Persist immediately, like the switches (2026-09-27: a Theme / brightness
    // mode choice used to be lost on reboot unless the footer Save was pressed).
    if (b->target != &cfg.screenRotation) {
        saveConfigToNVS(cfg);
        lv_label_set_text(settingsStatusLabel, "Saved");
    }
    // Rotation can't apply live (see AppConfig.h's screenRotation comment —
    // both screens are laid out once at boot for whichever orientation was
    // active then) — say so immediately rather than let the button silently
    // do nothing until the next restart.
    if (b->target == &cfg.screenRotation) {
        saveConfigToNVS(cfg);
        lv_label_set_text(settingsStatusLabel, "Rotation saved. Restart to apply.");
        if (restartConfirmOverlay) {
            lv_obj_clear_flag(restartConfirmOverlay, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void addChoiceRow(lv_obj_t *parent, int &y, const char *name, float *target, const char *const *labels,
                          int count) {
    lv_obj_t *nameLbl = lv_label_create(parent);
    lv_label_set_text(nameLbl, name);
    lv_obj_set_style_text_color(nameLbl, lv_color_hex(0xCCD6E0), 0);
    lv_obj_set_pos(nameLbl, 4, y);
    y += 15;

    int parentW = lv_obj_get_content_width(parent) - 8; // content area, minus room for the scrollbar
    const int gap = 4, leftMargin = 4, rightMargin = 4;
    int btnW = (parentW - leftMargin - rightMargin - gap * (count - 1)) / count;
    if (btnW > 90) btnW = 90; // don't stretch to absurd width on a wide landscape panel
    const int btnH = 22;

    ChoiceBinding *b = &choiceBindings[choiceCount++];
    b->count = count;
    b->target = target;
    int sel = (int)(*target);
    for (int i = 0; i < count; i++) {
        lv_obj_t *btn = lv_btn_create(parent);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
        lv_obj_set_size(btn, btnW, btnH);
        lv_obj_set_pos(btn, leftMargin + i * (btnW + gap), y);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x2E3B4E), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x2F7CE0), LV_STATE_CHECKED);
        if (i == sel) lv_obj_add_state(btn, LV_STATE_CHECKED);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, labels[i]);
        lv_obj_center(lbl);
        lv_obj_add_event_cb(btn, onChoiceBtnClicked, LV_EVENT_CLICKED, b);
        b->btns[i] = btn;
    }
    y += btnH + 6;
}

static void addSwitchRow(lv_obj_t *parent, int &y, const char *name, bool *target) {
    // Switches don't need the slider's stacked-row treatment — the switch
    // widget itself is small regardless of panel width, and every switch
    // label in this app is short enough to clear even a ~220px-wide
    // portrait column before the switch's own position below.
    int swX = lv_obj_get_content_width(parent) - 44 - 10; // fully inside the content area (was clipped)

    lv_obj_t *nameLbl = lv_label_create(parent);
    lv_label_set_text(nameLbl, name);
    lv_obj_set_style_text_color(nameLbl, lv_color_hex(0xCCD6E0), 0);
    lv_obj_set_pos(nameLbl, 4, y + 3);

    lv_obj_t *sw = lv_switch_create(parent);
    lv_obj_set_size(sw, 44, 22);
    lv_obj_set_pos(sw, swX, y);
    if (*target) lv_obj_add_state(sw, LV_STATE_CHECKED);

    SwitchBinding *b = &switchBindings[switchCount++];
    b->sw = sw;
    b->target = target;
    lv_obj_add_event_cb(sw, onSwitchChanged, LV_EVENT_VALUE_CHANGED, b);

    y += 26;
}

// Read-only diagnostic row (name + live value) — spec section 16.6/16.7
// "check" fields (GNSS fix, satellites, speed, Speed Map status). No
// binding/callback, unlike the slider/switch rows above: the value label is
// just handed back so refreshSensorsPanel() can update it on a timer. Kept
// as plain named lv_obj_t* pointers below rather than a generic array —
// there are only a handful and each needs different formatting.
// Value column position is relative to the parent's real width (not a
// fixed x=220) for the same portrait-width reasoning addSliderRow() gives —
// every value shown through this row is short enough (a number, "OK",
// "12.3 km/h") that it doesn't need the stacked treatment, just a value
// column that isn't hardcoded off the edge of a narrower panel.
static lv_obj_t *addReadonlyRow(lv_obj_t *parent, int &y, const char *name) {
    lv_obj_t *nameLbl = lv_label_create(parent);
    lv_label_set_text(nameLbl, name);
    lv_obj_set_style_text_color(nameLbl, lv_color_hex(0xCCD6E0), 0);
    lv_obj_set_pos(nameLbl, 4, y + 5);

    lv_obj_t *valLbl = lv_label_create(parent);
    lv_obj_set_style_text_color(valLbl, lv_color_white(), 0);
    lv_obj_set_pos(valLbl, lv_obj_get_content_width(parent) - 112, y + 5); // room for "NOT CONNECTED"-length values

    y += 22;
    return valLbl;
}

// For a value string long enough (e.g. the WiFi tab's "ON, IP=192.168.4.1")
// that it doesn't comfortably fit the ~160px-wide value column the
// two-column addReadonlyRow() layout above gives every other (short:
// "OK"/a number/"12.3 km/h") readonly row. Stacked instead: name on its own
// line, value below spanning nearly the full panel width.
static lv_obj_t *addWideReadonlyRow(lv_obj_t *parent, int &y, const char *name) {
    lv_obj_t *nameLbl = lv_label_create(parent);
    lv_label_set_text(nameLbl, name);
    lv_obj_set_style_text_color(nameLbl, lv_color_hex(0xCCD6E0), 0);
    lv_obj_set_pos(nameLbl, 4, y);
    y += 15;

    lv_obj_t *valLbl = lv_label_create(parent);
    lv_obj_set_style_text_color(valLbl, lv_color_white(), 0);
    lv_obj_set_pos(valLbl, 4, y);
    y += 19;
    return valLbl;
}

static lv_obj_t *gnssFixVal, *gnssSatsVal, *gnssSpeedRawVal, *gnssSpeedFilteredVal;
static lv_obj_t *wifiStatusVal; // Settings > WiFi tab — see refreshSensorsPanel()
static lv_obj_t *dataUpdateStatusVal = nullptr; // Settings > WiFi tab, online data update progress
static void onDataUpdateBtnClicked(lv_event_t *) {
    if (cfg.dataUpdateUrl[0] == '\0') {
        if (dataUpdateStatusVal) lv_label_set_text(dataUpdateStatusVal, "No data URL set (Config)");
        return;
    }
    if (dataUpdateStatusVal) lv_label_set_text(dataUpdateStatusVal, "Restarting to update...");
    dataUpdateSchedule(); // reboots into update mode (download runs there with RAM free for TLS)
}
// Settings > Sensors > "Speed Map" group — see refreshSensorsPanel(). Region/
// version come from speedLimitManagerGetInfo() (static once loaded at boot);
// the rest come from roadInfoSnapshot() (updates every ~500ms).
static lv_obj_t *speedMapStatusVal, *speedMapRegionVal, *speedMapVersionVal;
static lv_obj_t *speedMapLimitVal, *speedMapSourceVal, *speedMapMatchVal, *speedMapRoadIdVal;

static void refreshSensorsPanel(lv_timer_t *) {
    // Only worth doing while Settings is actually the screen on-screen —
    // same class of bug as Dashboard.cpp's simTimerCb (fixed alongside
    // this, user-reported 2026-09-15 laggy Settings interactions): a timer
    // that keeps doing work for a screen nobody's looking at just steals
    // CPU from whatever IS being interacted with.
    if (lv_screen_active() != settingsScreen) return;

    GnssSnapshot gnss = gnssSnapshot();

    // Three states — see Dashboard.cpp's gnssStatusWord for why "no fix
    // yet" (SEARCHING, normal while cold-starting/indoors) must read
    // differently from "nothing received at all" (FAULT, an actual
    // wiring/power/baud problem). User-reported confusion 2026-09-15.
    if (gnss.fix) {
        lv_label_set_text(gnssFixVal, "OK");
        lv_obj_set_style_text_color(gnssFixVal, lv_color_hex(0x33CC66), 0);
    } else if (gnss.linkAlive) {
        lv_label_set_text(gnssFixVal, "SEARCHING");
        lv_obj_set_style_text_color(gnssFixVal, lv_color_hex(0xE0C020), 0);
    } else {
        lv_label_set_text(gnssFixVal, "FAULT");
        lv_obj_set_style_text_color(gnssFixVal, lv_color_hex(0xFF3B30), 0);
    }
    lv_label_set_text_fmt(gnssSatsVal, "%d", gnss.satCount);

    // Floats -> snprintf into a buffer, never lv_label_set_text_fmt("%f"...)
    // directly: LV_USE_FLOAT=0 strips float support from LVGL's builtin
    // vsnprintf and it silently corrupts the varargs (LoadProhibited crash,
    // confirmed on real hardware 2026-09-14 — see Dashboard.cpp/README).
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f km/h", (double)gnss.rawSpeedKmh);
    lv_label_set_text(gnssSpeedRawVal, buf);
    snprintf(buf, sizeof(buf), "%.1f km/h", (double)gnss.egoSpeedKmh);
    lv_label_set_text(gnssSpeedFilteredVal, buf);


    // WiFi is a QR-only screen now — all its live status (AP creds, STA
    // connection, update-available) is drawn straight onto the full-screen QR
    // overlay by refreshScanListIfOpen() while it's open. Nothing else on the
    // WiFi tab to update.
    refreshScanListIfOpen();

    // Speed Map — spec section 25. Region/version only change if the SD
    // card is swapped and the device rebooted, so a single mapLoaded check
    // gates all the "static" fields; the rest (limit/source/match/road)
    // come from the live 500ms RoadInfoSnapshot regardless.
    SpeedMapMetadata mapInfo;
    bool mapLoaded = speedLimitManagerGetInfo(mapInfo);
    if (mapLoaded) {
        lv_label_set_text(speedMapStatusVal, "CONNECTED");
        lv_obj_set_style_text_color(speedMapStatusVal, lv_color_hex(0x33CC66), 0);
        lv_label_set_text_fmt(speedMapRegionVal, "%.16s", mapInfo.region);
        lv_label_set_text_fmt(speedMapVersionVal, "%.16s", mapInfo.mapVersion);
    } else {
        lv_label_set_text(speedMapStatusVal, "NOT CONNECTED");
        lv_obj_set_style_text_color(speedMapStatusVal, lv_color_hex(0x7C8A9A), 0);
        lv_label_set_text(speedMapRegionVal, "--");
        lv_label_set_text(speedMapVersionVal, "--");
    }

    RoadInfoSnapshot road = roadInfoSnapshot();
    if (road.valid) {
        char lb[16];
        snprintf(lb, sizeof(lb), "%.0f km/h", (double)road.speedLimitKmh); // libc: LVGL fmt has no %f
        lv_label_set_text(speedMapLimitVal, lb);
    } else {
        lv_label_set_text(speedMapLimitVal, "--");
    }
    lv_label_set_text(speedMapSourceVal, road.mapLoaded ? speedSourceStr(road.source) : "--");
    if (road.mapLoaded && road.roadId != 0) {
        // Confidence -> HIGH/MEDIUM/LOW bucket, spec section 16 — the exact
        // 0.0-1.0 number is available via /api/speedmap/debug for anyone
        // who wants it, not shown on this screen to keep it scannable.
        const char *bucket = road.confidence >= 0.8f ? "HIGH" : (road.confidence >= 0.5f ? "MEDIUM" : "LOW");
        char mb[32];
        snprintf(mb, sizeof(mb), "%s (%.2f)", bucket, (double)road.confidence);
        lv_label_set_text(speedMapMatchVal, mb);
        lv_label_set_text_fmt(speedMapRoadIdVal, "%lu", (unsigned long)road.roadId);
    } else {
        lv_label_set_text(speedMapMatchVal, "--");
        lv_label_set_text(speedMapRoadIdVal, "--");
    }
}

static void onBackToDashboard(lv_event_t *) { lv_screen_load(dashboardScreen); }

// Auto-return to the Dashboard after 5s of no touch anywhere (user-requested
// 2026-09-15) — reads Dashboard.cpp's shared touch timestamp rather than
// tracking its own, since there's only one touch source for the whole app;
// wakeScreen() already updates it on every touch regardless of which screen
// is showing. Checked once a second, not on every tick: a 5s timeout only
// needs ~1s granularity, and this runs even while idle on Settings so it
// isn't worth being more precise than a human would notice.
static void checkIdleReturnToDashboard(lv_timer_t *) {
    if (lv_screen_active() != settingsScreen) return;
    // Do NOT auto-exit while the user is in the WiFi menu (user-requested
    // 2026-09-26). Scanning, picking a network and typing a password on the
    // on-screen keyboard legitimately take longer than the 5s idle window, and
    // getting bounced back to the Dashboard mid-entry is exactly what the user
    // hit. Pause the timer for the whole WiFi tab, plus whenever the on-screen
    // keyboard or the full-screen WiFi setup overlay is open (both only appear
    // from WiFi flows, and the same "don't interrupt text entry" reasoning
    // applies to any field).
    // The WiFi (QR) screen gets a longer window instead: back to the Dashboard
    // after 1 minute without a touch (2026-09-27, "Setting/WiFi quay ve man hinh
    // home sau 1 phut") — but never while a phone is transferring data.
    bool wifiOpen = wifiScanOverlay && !lv_obj_has_flag(wifiScanOverlay, LV_OBJ_FLAG_HIDDEN);
    if (g_activeCategory == kCategoryWifi || wifiOpen) {
        static const uint32_t kWifiIdleTimeoutMs = 60000;
        if (wifiOpen && !updateApiBusy() && millis() - lastTouchAtMs() > kWifiIdleTimeoutMs) {
            Serial.println("[ui] WiFi screen idle 60 s -> Dashboard");
            onWifiScanClose(nullptr);
        }
        return;
    }
    if (wifiKeyboard && !lv_obj_has_flag(wifiKeyboard, LV_OBJ_FLAG_HIDDEN)) return;
    static const uint32_t kIdleTimeoutMs = 5000;
    if (millis() - lastTouchAtMs() > kIdleTimeoutMs) {
        lv_screen_load(dashboardScreen);
    }
}

// Save is confirmed through a modal rather than writing straight away: the
// settings being written include the safety thresholds, and NVS writes have
// limited endurance, so an accidental brush of the button should not persist.
static lv_obj_t *confirmOverlay;

static void closeConfirm(lv_event_t *) { lv_obj_add_flag(confirmOverlay, LV_OBJ_FLAG_HIDDEN); }

static void onConfirmSaveYes(lv_event_t *) {
    saveConfigToNVS(cfg);
    lv_obj_add_flag(confirmOverlay, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(settingsStatusLabel, "Saved to NVS");
    Serial.println("[uidemo] config saved to NVS");
}

static void onSaveConfig(lv_event_t *) { lv_obj_clear_flag(confirmOverlay, LV_OBJ_FLAG_HIDDEN); }

// Second, separate overlay for the footer's "Restart" button (added
// 2026-09-22) — a real device reboot is more disruptive than anything else
// on this screen, including Save above, yet used to fire on a single tap
// with NO confirmation at all, and with a deliberately ENLARGED touch
// hit-area on top of that (see restartBtn's own comment — a touch-accuracy
// accommodation). Confirmed on real hardware: the pre-existing touch
// controller glitch (TouchTask.cpp's "stuck" bus-reset recovery) was
// spuriously landing in that enlarged zone and silently triggering a real
// ESP.restart() — which looks exactly like a random crash from the
// driver's seat, not an accidental button press. A second stray touch
// hitting this modal's own Restart button too (normal-sized, not enlarged)
// is far less likely than one hitting the original button's 20px-padded
// zone, so this closes the actual gap rather than just relocating it.
// restartConfirmOverlay declared at file top
static void closeRestartConfirm(lv_event_t *) { lv_obj_add_flag(restartConfirmOverlay, LV_OBJ_FLAG_HIDDEN); }
static void onConfirmRestartYes(lv_event_t *) { ESP.restart(); }
static void onRestartBtnClicked(lv_event_t *) { lv_obj_clear_flag(restartConfirmOverlay, LV_OBJ_FLAG_HIDDEN); }

static void buildRestartConfirmOverlay(lv_obj_t *parent) {
    restartConfirmOverlay = lv_obj_create(parent);
    lv_obj_set_pos(restartConfirmOverlay, 0, 0);
    lv_obj_set_size(restartConfirmOverlay, gfx->width(), gfx->height());
    lv_obj_set_style_bg_color(restartConfirmOverlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(restartConfirmOverlay, LV_OPA_60, 0);
    lv_obj_set_style_border_width(restartConfirmOverlay, 0, 0);
    lv_obj_set_style_radius(restartConfirmOverlay, 0, 0);
    lv_obj_clear_flag(restartConfirmOverlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(restartConfirmOverlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *box = lv_obj_create(restartConfirmOverlay);
    lv_obj_set_size(box, 300, 130);
    lv_obj_center(box);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x1B222A), 0);
    lv_obj_set_style_border_color(box, lv_color_hex(0x3A4A5C), 0);
    lv_obj_set_style_radius(box, 8, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *msg = lv_label_create(box);
    lv_label_set_text(msg, "Restart the device now?");
    lv_obj_set_style_text_color(msg, lv_color_white(), 0);
    lv_obj_align(msg, LV_ALIGN_TOP_MID, 0, 6);

    lv_obj_t *noBtn = lv_button_create(box);
    lv_obj_set_size(noBtn, 110, 36);
    lv_obj_align(noBtn, LV_ALIGN_BOTTOM_LEFT, 0, -4);
    lv_obj_set_style_bg_color(noBtn, lv_color_hex(0x44505C), 0);
    lv_obj_set_ext_click_area(noBtn, 10);
    lv_obj_add_event_cb(noBtn, closeRestartConfirm, LV_EVENT_CLICKED, NULL);
    lv_obj_t *noLbl = lv_label_create(noBtn);
    lv_label_set_text(noLbl, "Cancel");
    lv_obj_center(noLbl);

    lv_obj_t *yesBtn = lv_button_create(box);
    lv_obj_set_size(yesBtn, 110, 36);
    lv_obj_align(yesBtn, LV_ALIGN_BOTTOM_RIGHT, 0, -4);
    lv_obj_set_style_bg_color(yesBtn, lv_color_hex(0x8A3A3A), 0);
    lv_obj_set_ext_click_area(yesBtn, 10);
    lv_obj_add_event_cb(yesBtn, onConfirmRestartYes, LV_EVENT_CLICKED, NULL);
    lv_obj_t *yesLbl = lv_label_create(yesBtn);
    lv_label_set_text(yesLbl, "Restart");
    lv_obj_center(yesLbl);
}

static void buildConfirmOverlay(lv_obj_t *parent) {
    confirmOverlay = lv_obj_create(parent);
    lv_obj_set_pos(confirmOverlay, 0, 0);
    lv_obj_set_size(confirmOverlay, gfx->width(), gfx->height());
    lv_obj_set_style_bg_color(confirmOverlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(confirmOverlay, LV_OPA_60, 0);
    lv_obj_set_style_border_width(confirmOverlay, 0, 0);
    lv_obj_set_style_radius(confirmOverlay, 0, 0);
    lv_obj_clear_flag(confirmOverlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(confirmOverlay, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *box = lv_obj_create(confirmOverlay);
    lv_obj_set_size(box, 300, 130);
    lv_obj_center(box);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x1B222A), 0);
    lv_obj_set_style_border_color(box, lv_color_hex(0x3A4A5C), 0);
    lv_obj_set_style_radius(box, 8, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *msg = lv_label_create(box);
    lv_label_set_text(msg, "Save settings to memory?");
    lv_obj_set_style_text_color(msg, lv_color_white(), 0);
    lv_obj_align(msg, LV_ALIGN_TOP_MID, 0, 6);

    lv_obj_t *noBtn = lv_button_create(box);
    lv_obj_set_size(noBtn, 110, 36);
    lv_obj_align(noBtn, LV_ALIGN_BOTTOM_LEFT, 0, -4);
    lv_obj_set_style_bg_color(noBtn, lv_color_hex(0x44505C), 0);
    lv_obj_set_ext_click_area(noBtn, 10);
    lv_obj_add_event_cb(noBtn, closeConfirm, LV_EVENT_CLICKED, NULL);
    lv_obj_t *noLbl = lv_label_create(noBtn);
    lv_label_set_text(noLbl, "Cancel");
    lv_obj_center(noLbl);

    lv_obj_t *yesBtn = lv_button_create(box);
    lv_obj_set_size(yesBtn, 110, 36);
    lv_obj_align(yesBtn, LV_ALIGN_BOTTOM_RIGHT, 0, -4);
    lv_obj_set_style_bg_color(yesBtn, lv_color_hex(0x2E7D4F), 0);
    lv_obj_set_ext_click_area(yesBtn, 10);
    lv_obj_add_event_cb(yesBtn, onConfirmSaveYes, LV_EVENT_CLICKED, NULL);
    lv_obj_t *yesLbl = lv_label_create(yesBtn);
    lv_label_set_text(yesLbl, "Save");
    lv_obj_center(yesLbl);
}

static void onRestoreDefaults(lv_event_t *) {
    cfg = AppConfig();
    clampConfig(cfg);
    applyConfig();
    for (int i = 0; i < sliderCount; i++) {
        SliderBinding &b = sliderBindings[i];
        lv_slider_set_value(b.slider, (int32_t)((*b.target) * b.divisor), LV_ANIM_OFF);
        char buf[24];
        fmtSettingVal(buf, sizeof(buf), *b.target, b.unit);
        lv_label_set_text(b.valLabel, buf);
    }
    for (int i = 0; i < switchCount; i++) {
        SwitchBinding &b = switchBindings[i];
        if (*b.target) lv_obj_add_state(b.sw, LV_STATE_CHECKED);
        else lv_obj_clear_state(b.sw, LV_STATE_CHECKED);
    }
    // WiFi SSID/password aren't in sliderBindings/switchBindings (see
    // onWifiSwitchChanged()'s comment) — refreshed by hand. Password field
    // stays blank, same as every other time it's displayed (see
    // wifiPasswordTa's own build comment) — cfg.wifiPassword IS reset
    // underneath, just never echoed into the field.
    // Null-guarded (2026-09-26): the WiFi tab's text fields no longer exist (the
    // tab is the QR screen now) — tapping "Defaults" crashed (LoadProhibited).
    if (wifiSsidTa) lv_textarea_set_text(wifiSsidTa, cfg.wifiSsid);
    if (wifiPasswordTa) lv_textarea_set_text(wifiPasswordTa, "");
    lv_label_set_text(settingsStatusLabel, "Restored defaults");
    Serial.println("[uidemo] config restored to defaults (not yet saved)");
}

// Category menu: left nav rail + one content panel per category, all
// pre-built and toggled via LV_OBJ_FLAG_HIDDEN (no rebuild/flicker on
// switching). Whole screen fits with no scrolling; Save/Defaults stay in a
// fixed footer visible from every category.
static const int kCategoryCount = 5;
static lv_obj_t *categoryPanels[kCategoryCount];
static lv_obj_t *navButtons[kCategoryCount];

static void onWifiScanOpen(lv_event_t *); // fwd decl — selectCategory opens the QR screen

void settingsSyncSwitches() {
    for (int i = 0; i < switchCount; i++) {
        SwitchBinding &b = switchBindings[i];
        if (*b.target) lv_obj_add_state(b.sw, LV_STATE_CHECKED);
        else lv_obj_clear_state(b.sw, LV_STATE_CHECKED);
    }
}

static void selectCategory(int idx) {
    g_activeCategory = idx;
    // The WiFi tab has no on-screen controls anymore — selecting it goes straight
    // to the full-screen QR setup screen (phone joins the AP + configures on the
    // web; the device then auto-connects + checks for updates).
    if (idx == kCategoryWifi && wifiScanOverlay) {
        onWifiScanOpen(nullptr);
        return;
    }
    for (int i = 0; i < kCategoryCount; i++) {
        if (i == idx) {
            lv_obj_clear_flag(categoryPanels[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(navButtons[i], lv_color_hex(0x2E4A66), 0);
        } else {
            lv_obj_add_flag(categoryPanels[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_style_bg_color(navButtons[i], lv_color_hex(0x151C24), 0);
        }
    }
}
static void onNavCategory(lv_event_t *e) { selectCategory((int)(intptr_t)lv_event_get_user_data(e)); }

lv_obj_t *settingsScreen;

void buildSettingsScreen() {
    settingsScreen = lv_obj_create(NULL);
    // Pure black, not 0x0B0F14 — same request/reasoning as Dashboard.cpp's
    // applyTheme() night background (2026-09-16, screen longevity/glare).
    lv_obj_set_style_bg_color(settingsScreen, lv_color_hex(0x000000), 0);
    lv_obj_set_style_pad_all(settingsScreen, 0, 0);
    lv_obj_clear_flag(settingsScreen, LV_OBJ_FLAG_SCROLLABLE);

    // scrW/scrH, not hardcoded 480/320 — Settings, like Dashboard.cpp, now
    // has to fit whichever orientation is currently active (AppConfig.h's
    // screenRotation). Unlike Dashboard, Settings keeps the SAME
    // header/nav-rail/content-panel/footer structure for every orientation
    // (least risky adaptation — no new interaction model to design/verify);
    // only the row-builders below (addSliderRow() etc.) branch on the
    // resulting content panel width, which is what actually differs.
    int scrW = gfx->width(), scrH = gfx->height();
    const int HEADER_H = 26, FOOTER_H = 34;
    const int NAV_W = 96;
    const int BODY_TOP = HEADER_H, BODY_H = scrH - HEADER_H - FOOTER_H;

    // Header
    lv_obj_t *header = lv_obj_create(settingsScreen);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_size(header, scrW, HEADER_H);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x151C24), 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_radius(header, 0, 0);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(header);
    lv_label_set_text(title, "Settings");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 10, 0);

    lv_obj_t *backBtn = lv_button_create(header);
    lv_obj_set_size(backBtn, 64, 20);
    lv_obj_align(backBtn, LV_ALIGN_RIGHT_MID, -6, 0);
    // This button sits inside a thin bar right at the very top edge of the
    // panel — the least accurate/reliable region of a capacitive touch
    // sensor (edge effects). Real-hardware complaint 2026-09-14: "không
    // Back hay Save được" (Back/Save unresponsive). A bigger *touch* target
    // than the *visual* button, rather than enlarging the button itself,
    // fixes the near-miss taps without changing the compact header look.
    lv_obj_set_ext_click_area(backBtn, 20);
    lv_obj_add_event_cb(backBtn, onBackToDashboard, LV_EVENT_CLICKED, NULL);
    lv_obj_t *backLbl = lv_label_create(backBtn);
    lv_label_set_text(backLbl, "< Back");
    lv_obj_center(backLbl);

    // Left nav rail
    lv_obj_t *navRail = lv_obj_create(settingsScreen);
    lv_obj_set_pos(navRail, 0, BODY_TOP);
    lv_obj_set_size(navRail, NAV_W, BODY_H);
    lv_obj_set_style_bg_color(navRail, lv_color_hex(0x0E141B), 0);
    lv_obj_set_style_border_width(navRail, 0, 0);
    lv_obj_set_style_radius(navRail, 0, 0);
    lv_obj_set_style_pad_all(navRail, 4, 0);
    lv_obj_clear_flag(navRail, LV_OBJ_FLAG_SCROLLABLE);

    static const char *kCategoryNames[kCategoryCount] = {"Display", "Map", "Sensors", "WiFi", "Audio"};
    for (int i = 0; i < kCategoryCount; i++) {
        lv_obj_t *btn = lv_button_create(navRail);
        lv_obj_set_size(btn, NAV_W - 8, 42);
        lv_obj_set_pos(btn, 0, i * 48);
        lv_obj_set_style_radius(btn, 4, 0);
        lv_obj_set_ext_click_area(btn, 5); // small — buttons are stacked with only a 6px gap
        lv_obj_add_event_cb(btn, onNavCategory, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, kCategoryNames[i]);
        lv_obj_center(lbl);
        navButtons[i] = btn;
    }

    // Content panels (one per category, same rect, toggled via HIDDEN)
    for (int i = 0; i < kCategoryCount; i++) {
        lv_obj_t *panel = lv_obj_create(settingsScreen);
        lv_obj_set_pos(panel, NAV_W + 4, BODY_TOP);
        lv_obj_set_size(panel, scrW - NAV_W - 4, BODY_H);
        // Root cause of the 2026-09-16 "khong thay nut nhan" / mispositioned-
        // switch bug: lv_obj_get_width()/height() only reflect a *pending*
        // lv_obj_set_size() after the next redraw (LVGL's own documented
        // behavior, lv_obj_pos.h) — this screen is built entirely inside
        // setup(), before lv_timer_handler() has ever run once, so every
        // row-builder below that reads lv_obj_get_width(parent) (addSliderRow,
        // addSwitchRow, addChoiceRow, ...) was silently reading back 0 the
        // whole time since the rotation refactor introduced width-aware rows,
        // not just for the new Theme/Rotation buttons. Forcing the layout
        // commit here, once per panel right after sizing it, makes every
        // later lv_obj_get_width(categoryPanels[i]) call return the real
        // value instead.
        lv_obj_update_layout(panel);
        lv_obj_set_style_bg_color(panel, lv_color_hex(0x000000), 0); // pure black — see settingsScreen's own comment above
        lv_obj_set_style_border_width(panel, 0, 0);
        lv_obj_set_style_pad_all(panel, 4, 0);
        // Display and WiFi both fit inside BODY_H with no scrolling. Sensors
        // (i==1) doesn't — the GNSS live-status rows plus the Speed Map
        // diagnostics group (7 fields) genuinely don't fit in 260px, and
        // it's allowed to be long (spec section 25 calls it "for kiểm
        // tra/cấu hình," not the driving screen) rather than needing rows
        // trimmed to squeeze in. The old Radar/Safety tabs that used to also
        // need scrolling here are gone entirely (radar removed 2026-09-21).
        // Map (1) and Sensors (2) are taller than BODY_H and scroll. (Was "i == 2 ||
        // i == 3" — left over from before the Map tab was inserted at index 1, so the
        // Map tab couldn't scroll. WiFi (3) is just the QR overlay now.)
        if (i <= 2 || i == 4) { // Display, Map (+ Speed Map group), Sensors and Audio are taller than BODY_H
            lv_obj_set_scroll_dir(panel, LV_DIR_VER);
            lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_AUTO);
        } else {
            lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
        }
        lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN);
        categoryPanels[i] = panel;
    }

    int y;
    // Display tab (categoryPanels[0]) — brightness/dim/theme/rotation plus
    // the master alert-audio toggle (moved here from the old Safety tab,
    // which no longer exists — see cfg.audioEnabled's own AppConfig.h
    // comment for why it's still just a plain on/off, repurposed rather
    // than removed, when radar was taken out 2026-09-21).
    y = 4;
    // (Alert audio on/off + volume moved to the "Âm thanh" tab 2026-09-26.)
    addSliderRow(categoryPanels[0], y, "Brightness", &cfg.brightness, 5, 100, 1.0f, " %");
    // Backlight mode (2026-09-26): Auto caps the backlight at 50 % at night
    // (same GNSS sunrise/sunset as the Auto theme); Manual = always the slider.
    static const char *kBrightnessModeLabels[2] = {"Auto", "Manual"};
    addChoiceRow(categoryPanels[0], y, "Brightness mode", &cfg.brightnessMode, kBrightnessModeLabels, 2);
    {
        lv_obj_t *hint = lv_label_create(categoryPanels[0]);
        lv_label_set_text(hint, "Auto: 50% brightness at night");
        lv_obj_set_style_text_color(hint, lv_color_hex(0x7C8A9A), 0);
        lv_obj_set_pos(hint, 4, y);
        y += 22;
    }
    // Label updated 2026-09-16 alongside the gate itself changing from
    // touch-idle to vehicle-stationary time (see Dashboard.cpp) — "stopped"
    // says what actually starts the timer now.
    addSliderRow(categoryPanels[0], y, "Dim after stopped", &cfg.autoDimMin, 0, 30, 1.0f, " min");
    // Theme (user-requested 2026-09-15): 0=Auto follows GNSS.cpp's real
    // sunrise/sunset calc (unchanged default behavior), 1=Light/2=Dark force
    // it either way. Applies live — ui/Dashboard.cpp's refreshDashboard()
    // re-checks cfg.themeMode on the same tick it already checks
    // gnss.daytime, no restart needed (see applyTheme()'s call site there).
    // Choice buttons, not a slider (user-reported 2026-09-16, "nut gat chua
    // hop ly") — a 3-way enum on a slider hid which option was active.
    static const char *kThemeLabels[3] = {"Auto", "Light", "Dark"};
    addChoiceRow(categoryPanels[0], y, "Theme", &cfg.themeMode, kThemeLabels, 3);
    // Rotation: does NOT apply live — see onChoiceBtnClicked()'s special
    // case for this field and AppConfig.h's screenRotation comment for why
    // (both Dashboard and Settings are laid out once at boot for whichever
    // orientation is active then). 0/1/2/3 matches Arduino_GFX's own
    // rotation convention exactly (display/DisplayDriver.cpp passes this
    // straight through) — 1 is today's boot default (landscape). Choice
    // buttons rather than a slider for the same reason as Theme above, and
    // because a held slider drag was what triggered the touch controller's
    // known stuck-bus quirk (2026-09-14) during the user's own testing.
    static const char *kRotationLabels[4] = {"0", "90", "180", "270"};
    addChoiceRow(categoryPanels[0], y, "Rotation", &cfg.screenRotation, kRotationLabels, 4);
#ifdef VIETHUD_P4
    { // 5-point touch calibration (touch/TouchTaskP4.cpp); runs from the main loop
        lv_obj_t *btn = lv_button_create(categoryPanels[0]);
        lv_obj_set_pos(btn, 4, y + 4);
        lv_obj_set_size(btn, 220, 34);
        lv_obj_t *l = lv_label_create(btn);
        lv_label_set_text(l, "Hiệu chuẩn cảm ứng");
        lv_obj_center(l);
        lv_obj_add_event_cb(btn, [](lv_event_t *) { gTouchCalRequested = true; }, LV_EVENT_CLICKED, NULL);
        y += 44;
    }
#endif

    // Audio tab (categoryPanels[4], 2026-09-26): master switch + volume, then
    // which alert types speak. Holding the Dashboard ~2.5 s flips the master.
    {
        lv_obj_t *ap = categoryPanels[4];
        int ya = 4;
        addSwitchRow(ap, ya, "Alert sound", &cfg.audioEnabled);
        addSliderRow(ap, ya, "Volume", &cfg.audioVolume, 0, 100, 1.0f, " %");
        lv_obj_t *hint = lv_label_create(ap);
        lv_label_set_text(hint, "Hold the main screen 2.5 s to toggle sound.\nPlay sound for:");
        lv_obj_set_style_text_color(hint, lv_color_hex(0x7C8A9A), 0);
        lv_obj_set_pos(hint, 4, ya);
        ya += 40;
        addSwitchRow(ap, ya, "Overspeed", &cfg.audioOverspeed);
        addSwitchRow(ap, ya, "Camera", &cfg.audioCamera);
        addSwitchRow(ap, ya, "Speed limit ahead", &cfg.audioLimitAhead);
        addSwitchRow(ap, ya, "Residential area", &cfg.audioResident);
        addSwitchRow(ap, ya, "No overtaking", &cfg.audioNoOvertake);
        addSwitchRow(ap, ya, "Toll booth", &cfg.audioToll);
        addSwitchRow(ap, ya, "Traffic light", &cfg.audioLight);
        addSwitchRow(ap, ya, "Danger zone", &cfg.audioDanger);
        addSwitchRow(ap, ya, "GPS / temperature", &cfg.audioSystem);
    }

    // -----------------------------------------------------------------
    // Map tab (categoryPanels[1]) — vector map display options
    // -----------------------------------------------------------------
    y = 4;
    // Vector map only (raster JPEG background removed 2026-09-26).
    addSwitchRow(categoryPanels[1], y, "Heading up (rotate map)", &cfg.mapHeadingUp);
    addSwitchRow(categoryPanels[1], y, "Vehicle trail", &cfg.showVehicleTrail);

    // Speed Map diagnostics (spec section 25) — offline microSD map-matching
    // status, see map/SpeedLimitManager.h. This group is why categoryPanels[1]
    // needs to be scrollable above: it doesn't fit in 260px alongside
    // everything already in this tab.
    y += 6;
    lv_obj_t *mapHeaderSpeed = lv_label_create(categoryPanels[1]);
    lv_label_set_text(mapHeaderSpeed, "Speed Map");
    lv_obj_set_style_text_color(mapHeaderSpeed, lv_color_hex(0x7C8A9A), 0);
    lv_obj_set_pos(mapHeaderSpeed, 4, y);
    y += 20;
    speedMapStatusVal = addReadonlyRow(categoryPanels[1], y, "Status");
    speedMapRegionVal = addReadonlyRow(categoryPanels[1], y, "Region");
    speedMapVersionVal = addReadonlyRow(categoryPanels[1], y, "Version");
    speedMapLimitVal = addReadonlyRow(categoryPanels[1], y, "Current limit");
    speedMapSourceVal = addReadonlyRow(categoryPanels[1], y, "Source");
    speedMapMatchVal = addReadonlyRow(categoryPanels[1], y, "Match");
    speedMapRoadIdVal = addReadonlyRow(categoryPanels[1], y, "Road ID");


    // Sensors tab (categoryPanels[2]) — real-hardware check/configure
    // (spec 16.6/16.7). GNSS (u-blox M10N, UART2) went real 2026-09-15;
    // radar (HLK-LD2451) went real 2026-09-16 and was removed entirely
    // 2026-09-21 (GPS-only VietHUD product) — the old Radar-status/
    // Radar-tracking rows that used to live here are gone with it.
    y = 4;
    // Trip logging (added 2026-09-16, see log/TripLogger.h).
    addSwitchRow(categoryPanels[2], y, "Trip logging (SD card)", &cfg.tripLoggingEnabled);
    y += 6; // extra breathing room before the real live-status rows below

    lv_obj_t *sensorsHeader1 = lv_label_create(categoryPanels[2]);
    lv_label_set_text(sensorsHeader1, "Live status");
    lv_obj_set_style_text_color(sensorsHeader1, lv_color_hex(0x7C8A9A), 0);
    lv_obj_set_pos(sensorsHeader1, 4, y);
    y += 20;
    gnssFixVal = addReadonlyRow(categoryPanels[2], y, "GNSS fix");
    gnssSatsVal = addReadonlyRow(categoryPanels[2], y, "Satellites");
    gnssSpeedRawVal = addReadonlyRow(categoryPanels[2], y, "Speed (raw)");
    gnssSpeedFilteredVal = addReadonlyRow(categoryPanels[2], y, "Speed (filtered)");

    y += 6;
    lv_obj_t *sensorsHeader2 = lv_label_create(categoryPanels[2]);
    lv_label_set_text(sensorsHeader2, "GNSS calibration");
    lv_obj_set_style_text_color(sensorsHeader2, lv_color_hex(0x7C8A9A), 0);
    lv_obj_set_pos(sensorsHeader2, 4, y);
    y += 20;
    addSliderRow(categoryPanels[2], y, "Speed filter smoothing", &cfg.gnssSpeedFilterAlpha, 5, 90, 100.0f, "");
    addSliderRow(categoryPanels[2], y, "Fix timeout", &cfg.gnssFixTimeoutS, 10, 100, 10.0f, " s");
    // GPS speed calibration + ahead-warning distances (user-requested
    // 2026-09-22, "hieu chinh toc do GPS, khoang cach canh bao toc do/
    // camera phia truoc") — see AppConfig.h's own comment on each field for
    // why calibration is a percentage and why only these two distances (not
    // every alert type) are exposed here. Both distance ranges capped at
    // 100m max/default 2026-09-22 ("khoang cach toi da de canh bao la 100m,
    // mac dinh la 100m") — see AppConfig.h's clampConfig() for the matching
    // clamp.
    addSliderRow(categoryPanels[2], y, "Speed calibration", &cfg.gnssSpeedCalibrationPct, -150, 150, 10.0f, " %");
    // Overspeed warning threshold (was un-tunable — the field existed in
    // AppConfig but no control reached it; exposed 2026-09-24). Warning fires
    // when egoSpeed > limit + this offset. 0..10 km/h.
    addSliderRow(categoryPanels[2], y, "Overspeed offset", &cfg.overspeedOffsetKmh, 0, 10, 1.0f, " km/h");
    // Fallback speed limit shown where the map can't resolve one (0 = off/"--").
    // Default 50 = VN urban baseline (user-requested 2026-09-24).
    addSliderRow(categoryPanels[2], y, "Default limit (unknown)", &cfg.defaultLimitKmh, 0, 90, 1.0f, " km/h");
    // NOTE: the old "Speed-limit-ahead dist" / "Camera warn dist" sliders were
    // removed 2026-09-24 — they did nothing. The lookahead uses a DYNAMIC,
    // speed-based warn distance (computeDynamicWarnDistance: ~100-600m scaling
    // with speed) in SpeedLimitManager.cpp, not cfg.aheadLimitWarnDistM/
    // cameraWarnDistM. Those cfg fields are now legacy/unused.


    // WiFi tab (2026-09-26): the whole WiFi menu is now the QR setup screen.
    // Selecting this tab immediately opens the full-screen QR overlay (see
    // selectCategory) where the phone joins the device AP and configures WiFi on
    // the web portal; the device then auto-connects + auto-checks for updates.
    // All on-screen AP/STA/scan/data-update controls were removed - no on-device
    // typing. This panel just holds a one-line fallback in case it is ever seen.
    {
        lv_obj_t *wifiHint = lv_label_create(categoryPanels[3]);
        lv_label_set_long_mode(wifiHint, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(wifiHint, lv_obj_get_width(categoryPanels[3]) - 8);
        lv_obj_set_style_text_color(wifiHint, lv_color_hex(0xCCD6E0), 0);
        lv_label_set_text(wifiHint, "Opening the QR code to connect a phone...");
        lv_obj_set_pos(wifiHint, 6, 8);
    }

    // NOT called eagerly here: buildSettingsScreen() runs before
    // sharedStateInit() in main_ui_demo.cpp's setup(), so
    // gnssSnapshot()'s mutex doesn't exist yet — an eager
    // call here crashed real hardware 2026-09-15 (xQueueSemaphoreTake
    // assert on a NULL queue, boot-looping). The 500ms timer's first tick
    // fires from loop() well after sharedStateInit() has run, so the rows
    // just start blank for under a second instead.
    lv_timer_create(refreshSensorsPanel, 500, NULL); // live values only need to be as fresh as a human reads them
    // Phone-update progress pill: runs on EVERY screen (refreshSensorsPanel bails
    // out unless Settings is showing), cheap no-op while no phone is updating.
    lv_timer_create(
        [](lv_timer_t *) {
            refreshBridgeToast();
            if (connToast && connToastUntilMs && (int32_t)(millis() - connToastUntilMs) >= 0) {
                connToastUntilMs = 0;
                lv_obj_add_flag(connToast, LV_OBJ_FLAG_HIDDEN);
            }
        },
        500, NULL);
    lv_timer_create(checkIdleReturnToDashboard, 1000, NULL);

    selectCategory(0);

    // Footer — always visible regardless of selected category
    lv_obj_t *footer = lv_obj_create(settingsScreen);
    lv_obj_set_pos(footer, 0, scrH - FOOTER_H);
    lv_obj_set_size(footer, scrW, FOOTER_H);
    lv_obj_set_style_bg_color(footer, lv_color_hex(0x151C24), 0);
    lv_obj_set_style_border_width(footer, 0, 0);
    lv_obj_set_style_radius(footer, 0, 0);
    lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);

    settingsStatusLabel = lv_label_create(footer);
    lv_obj_set_style_text_color(settingsStatusLabel, lv_color_hex(0x66CC88), 0);
    lv_label_set_text(settingsStatusLabel, "");
    lv_obj_align(settingsStatusLabel, LV_ALIGN_LEFT_MID, 12, 0);

    lv_obj_t *restoreBtn = lv_button_create(footer);
    lv_obj_set_size(restoreBtn, 90, 24);
    lv_obj_align(restoreBtn, LV_ALIGN_RIGHT_MID, -108, 0);
    lv_obj_set_ext_click_area(restoreBtn, 20); // see backBtn comment — bottom-edge bar, same fix
    lv_obj_add_event_cb(restoreBtn, onRestoreDefaults, LV_EVENT_CLICKED, NULL);
    lv_obj_t *restoreLbl = lv_label_create(restoreBtn);
    lv_label_set_text(restoreLbl, "Defaults");
    lv_obj_center(restoreLbl);

    // Convenience for the rotation slider's "Restart to apply rotation"
    // message (onSliderChanged()) — ESP.restart() directly rather than
    // asking the user to power-cycle by hand. Placed left of Defaults, same
    // 90px/click-area treatment; a bit snug against settingsStatusLabel in
    // portrait's narrower ~320px footer when that specific long message is
    // showing, but this button is otherwise blank/unused so it's a cosmetic
    // crowding at worst, not a functional collision.
    lv_obj_t *restartBtn = lv_button_create(footer);
    lv_obj_set_size(restartBtn, 70, 24);
    lv_obj_align(restartBtn, LV_ALIGN_RIGHT_MID, -208, 0);
    lv_obj_set_style_bg_color(restartBtn, lv_color_hex(0x44505C), 0);
    lv_obj_set_ext_click_area(restartBtn, 20);
    lv_obj_add_event_cb(restartBtn, onRestartBtnClicked, LV_EVENT_CLICKED, NULL);
    lv_obj_t *restartLbl = lv_label_create(restartBtn);
    lv_label_set_text(restartLbl, "Restart");
    lv_obj_center(restartLbl);

    lv_obj_t *saveBtn = lv_button_create(footer);
    lv_obj_set_size(saveBtn, 90, 24);
    lv_obj_align(saveBtn, LV_ALIGN_RIGHT_MID, -8, 0);
    lv_obj_set_style_bg_color(saveBtn, lv_color_hex(0x2E7D4F), 0);
    lv_obj_set_ext_click_area(saveBtn, 20);
    lv_obj_add_event_cb(saveBtn, onSaveConfig, LV_EVENT_CLICKED, NULL);
    lv_obj_t *saveLbl = lv_label_create(saveBtn);
    lv_label_set_text(saveLbl, "Save");
    lv_obj_center(saveLbl);

    // On-screen keyboard for the WiFi tab's SSID/password fields — hidden
    // until onWifiTaClicked() binds+shows it, moved to the foreground at
    // that point since it's created here (before buildConfirmOverlay) and
    // would otherwise sit behind it in z-order.
    wifiKeyboard = lv_keyboard_create(settingsScreen);
    lv_obj_set_size(wifiKeyboard, scrW, 150);
    lv_obj_set_pos(wifiKeyboard, 0, scrH - 150);
    lv_obj_add_flag(wifiKeyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(wifiKeyboard, onWifiKbReadyOrCancel, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(wifiKeyboard, onWifiKbReadyOrCancel, LV_EVENT_CANCEL, NULL);

    buildWifiScanOverlay(settingsScreen); // full-screen "WiFi setup" (scan + password) — see its own comment
    lv_obj_move_foreground(wifiKeyboard); // keep the keyboard above the overlay when both show

    buildConfirmOverlay(settingsScreen); // created last so it covers everything (and now the keyboard too)
    buildRestartConfirmOverlay(settingsScreen);
}

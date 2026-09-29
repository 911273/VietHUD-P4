#include "map/SpeedLimitManager.h"
#include <esp_task_wdt.h>
#include "Dashboard.h"
#include "map/MapRenderer.h"
#include "map/SdCardManager.h" // sdMgrExists() — full-sentence voice prompts present on the card?
#include "Settings.h" // settingsScreen, for the hold-to-open-Settings gesture
#include "core/AppConfig.h"
#include "core/NvsStore.h" // saveConfigToNVS()
#include "core/SharedState.h"
#include "display/DisplayDriver.h" // backlightWrite
#include "gnss/GNSS.h" // gnssMsSinceStationary() — auto-dim's vehicle-stationary gate
#include "icons/Icons.h"
#include "net/WebPortal.h"
#include "audio/AudioPlayer.h"

LV_FONT_DECLARE(lv_font_montserrat_speed); // big speed digits
#ifdef VIETHUD_P4
#include "driver/temperature_sensor.h" // IDF 5 driver — the legacy one aborts next to it
#else
#include "driver/temp_sensor.h" // ESP32-S3 die sensor with selectable range (readDieTempC)
#endif
LV_FONT_DECLARE(lv_font_vn_14); // Vietnamese-capable text font (Arial 14px, ASCII+VN); drop-in for montserrat_14
LV_FONT_DECLARE(lv_font_vn_20); // same at 20px — landscape top bar (matches the 20px bottom-corner readouts)

// Copies a UTF-8 road name into `out`, abbreviating a leading Vietnamese
// "Đường " to "Đ. " (user-requested 2026-09-24 — street names from the OSM DB
// are almost all prefixed "Đường ...", which overflows the narrow badge). The
// match is on the raw UTF-8 bytes of "Đường " so it is encoding-safe. Only the
// leading occurrence is abbreviated; the rest is copied verbatim.
static void abbreviateRoadName(const char *in, char *out, size_t outsz) {
    if (!in || !out || outsz == 0) return;
    static const char kDuong[] = "\xC4\x90\xC6\xB0\xE1\xBB\x9Dng "; // "Đường " in UTF-8
    const size_t kLen = sizeof(kDuong) - 1;
    if (strncmp(in, kDuong, kLen) == 0) {
        snprintf(out, outsz, "\xC4\x90. %s", in + kLen); // "Đ. " + remainder
    } else {
        strncpy(out, in, outsz - 1);
        out[outsz - 1] = '\0';
    }
}
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------
// VietHUD Dashboard — GPS-only offline speed-limit/camera/sign warning
// display. Radar (HLK-LD2451) and everything it drove (TTC/risk-color
// warning cell, per-target road panel, audio-gate hysteresis, tailgating/
// harsh-brake banners) was removed entirely 2026-09-21 — this project is no
// longer a forward-collision radar display, see the repo's own README for
// the product this became. The layout below is what's left after that
// removal, reflowed: [ speed + speed-limit sign | sign/camera alert card ],
// with a top status bar (GNSS/clock/settings) and a one-line bottom info
// bar (speed-map region/version + GNSS satellite count). Two decisions
// carried over from the original radar-era layout, both because the
// compiled-in LVGL font can't render what a nicer mockup would want
// (verified directly against lv_font_montserrat_14.c's cmap — ASCII +
// ~60 symbol codepoints only, no emoji, no Vietnamese diacritics):
//   - All labels stay English/ASCII (no custom Vietnamese font asset) —
//     the sign/camera alert card's own text uses plain-ASCII transliterated
//     Vietnamese (e.g. "KHU DONG DAN CU") for the same reason.
//   - Warning/sun/moon icons are small generated bitmaps (see
//     ui/icons/Icons.h for how and why) instead of font glyphs.
// GPS/Settings telltales keep using the built-in LV_SYMBOL_* glyphs
// (LV_SYMBOL_GPS/SETTINGS/WIFI) — those aren't emoji, they're already part
// of the compiled font.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// Screen burn-in / image retention mitigation
// ---------------------------------------------------------------------
// Two independent measures, both required because the Dashboard keeps
// several high-contrast elements in a fixed spot for the whole trip (top
// status bar, column dividers, bottom info bar):
//   1. Pixel shift — the entire dashboard is nudged a couple of pixels on a
//      slow cycle so no edge sits on the same physical pixels for hours.
//   2. Auto-dim — backlight drops to a low level after the vehicle has been
//      continuously stationary for a configurable period (default 3 min —
//      user-requested 2026-09-16, "che do tu giam do sang man hinh chi duoc
//      thuc hien khi xe khong chuyen dong sau 3 phut"); a touch OR the
//      vehicle moving again both restore it. Lower luminance dramatically
//      slows image retention on IPS panels. Gated on GNSS speed
//      (gnssMsSinceStationary(), GNSS.h) rather than touch-idle time alone —
//      never dim while the driver is relying on the warning display during
//      an actual drive, even if they never touch the screen.
static const int kPixelShiftOffsets[4][2] = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
static int pixelShiftIdx = 0;
static uint32_t lastTouchMs = 0;
static bool screenDimmed = false;
static bool gThermalDimmed = false; // die temp critical -> backlight forced low (see the temperature block)
// Night per GNSS sunrise/sunset, cached by refreshDashboard(). applyConfig() is
// also called early in setup(), BEFORE sharedStateInit() creates the mutex
// gnssSnapshot() takes — so it must not call gnssSnapshot() itself.
static bool gIsNight = false;

// The ONE place that decides the backlight level, in priority order:
// thermal protection > stationary auto-dim > night cap (Auto mode) > the
// configured brightness.
void applyConfig() {
    float level = cfg.brightness;
    if (cfg.brightnessMode < 0.5f && gIsNight && level > AppConfig::kNightBrightnessPct)
        level = AppConfig::kNightBrightnessPct; // Auto: night -> at most 50 %
    if (screenDimmed) level = 12.0f;
    if (gThermalDimmed) level = 16.0f;       // ~16 % to shed heat
    backlightWrite((uint32_t)(level / 100.0f * 255.0f));
    audioSetVolume((uint8_t)cfg.audioVolume); // push the configured speaker volume to the audio driver
}

void wakeScreen() {
    lastTouchMs = millis();
    if (screenDimmed) {
        screenDimmed = false;
        applyConfig();
    }
}

uint32_t lastTouchAtMs() { return lastTouchMs; }

// Semantic risk color — the ONLY place red/amber/purple are used to mean
// "something needs the driver's attention" (speeding, an upcoming sign).
// System telltales (GPS/Settings) use the cyan accent below instead,
// precisely so a "system OK" color is never confused with a "warning"
// color (spec section 14.2, carried over from the radar-era Dashboard).
#define ACCENT_COLOR 0x3FCAD6

// applyTheme()'s last-computed "normal" text color for speedLabel —
// remembered so refreshDashboard() can restore it after an overspeed
// tick's red override without needing to know which theme is active itself.
static lv_color_t currentPrimaryTextColor = lv_color_white();

// Day/Night palettes (2026-09-27: "Theme chua hoat dong: Auto/Light/Dark" —
// applyTheme() used to ignore its argument and always paint the dark theme).
// Light = light-grey map with darker roads and dark text for bright daylight;
// Dark = the original near-black cockpit look. Risk colours (red/amber/purple),
// the P.127 speed sign and the GNSS status colours are the same in both.
struct ThemePalette {
    uint32_t bg;          // screen + map background
    uint32_t roadCurrent, roadMajor, roadMain, roadSmall;
    uint32_t primaryText; // speed number, clock
    uint32_t captionText;
    uint32_t barLine;     // top/bottom bar divider line
    uint32_t badgeBg, badgeBorder, badgeText, accent;
    uint32_t cardBg, cardBorder, cardText, cardTrack;
    uint32_t neutral;     // temperature (normal), audio icon (on)
    uint32_t toastBg, toastText;
};
static const ThemePalette kDarkPalette = {
    0x04060A, 0x7FD3E0, 0xB4BCC6, 0x8A94A0, 0x616A76, 0xFFFFFF, 0xD0E0F0, 0x182232,
    0x0C1522, 0x1F314A, 0xF0F4F8, 0x00E5FF, 0x0C1420, 0x28384C, 0xFFFFFF, 0x182434,
    0x93A0AE, 0x151C24, 0xFFFFFF};
static const ThemePalette kLightPalette = {
    0xE6EAEE, 0x00838F, 0x4A5561, 0x6E7985, 0x98A2AD, 0x0A0E14, 0x2A3440, 0xB8C2CC,
    0xFFFFFF, 0xC2CBD4, 0x10151C, 0x00838F, 0xFFFFFF, 0xC2CBD4, 0x10151C, 0xD8DEE4,
    0x4A5561, 0xFFFFFF, 0x10151C};
static bool gLightTheme = false;
static const ThemePalette &pal() { return gLightTheme ? kLightPalette : kDarkPalette; }
static bool gThemeRepaintMap = false; // map canvas must be repainted in the new colours

static void applyTheme(bool daytime); // defined below buildDashboard(), which calls it once at the end to set the initial theme

// ---------------------------------------------------------------------
// Screen + layout constants
// ---------------------------------------------------------------------
lv_obj_t *dashboardScreen;
static lv_obj_t *dashRoot; // everything sits in here so it can be pixel-shifted

// Landscape-only layout constants (rotation 1 or 3 — see AppConfig.h's
// screenRotation — both produce this exact 480x320 logical canvas, per
// Arduino_GFX's own rotation convention, so hardcoding is safe here the same
// way it always was before rotation became configurable). Portrait
// (rotation 0/2, 320x480) has its own separate constants inside
// buildDashboardPortrait() — the two layouts are different ARRANGEMENTS of
// the same information, not one parametric layout stretched to fit either
// shape (a 3-column layout doesn't sensibly reflow into a tall narrow
// screen; see the plan this was built from for why a second, purpose-built
// layout was chosen over trying to make one layout adapt to both).
//
// LS_-prefixed (2026-09-22, landscape-corruption investigation rewrite):
// buildDashboardPortrait() has its OWN locals named scrW/scrH/etc — same
// semantic role, different numbers. They were never actually mixed up
// anywhere (checked), but the shared/unprefixed names were flagged as a
// real hazard for a future edit, so this rewrite renames the landscape set
// to make that class of mistake impossible rather than just improbable.
#ifdef VIETHUD_P4
// P4 800x480 panel: design units x1.5 (ui/UiScale.h) -> 533x320 design screen;
// the extra width goes to the map column.
static const int LS_SCR_W = 533, LS_SCR_H = 320;
#else
static const int LS_SCR_W = 480, LS_SCR_H = 320;
#endif
static const int LS_TOP_H = 30, LS_BOTTOM_H = 26;
// Split 50/50 (user-requested 2026-09-16, "chia doi man hinh... phan hien
// thi xe sang 1 ben, nua man hinh con lai la cac thong so") — replaces the
// old 96|288|96 three-column split. LS_PARAM_COL (left half) holds every
// number/status readout (speed, speed limit sign, warning, TTC) stacked in
// ONE evenly-spaced column (retuned 2026-09-16, "bo tri hop ly, can bang va
// deu nhau" — an earlier two-sub-column attempt left mismatched gaps, see
// buildDashboardLandscape()'s own comment at that block); LS_ROAD_COL (right
// half) is just the road/target view, now much bigger than the old 288px-
// wide middle column.
static const int LS_PARAM_COL_W = 240, LS_ROAD_COL_W = LS_SCR_W - LS_PARAM_COL_W;
static const int LS_ROAD_COL_X = LS_PARAM_COL_W;
static const int LS_COL_TOP = LS_TOP_H, LS_COL_H = LS_SCR_H - LS_TOP_H - LS_BOTTOM_H;

// --- Map Canvas & Digital Zoom ---
static lv_obj_t *mapCanvas = nullptr;
static uint16_t *mapCanvasBuf = nullptr;
static lv_obj_t *egoArrow = nullptr;
static lv_obj_t *egoHalo = nullptr;
// North indicator (user-requested 2026-09-24): a short line + "N" that always
// points to true North. In heading-up mode the map rotates, so North swings
// around the car; this line shows it. Lives in a fixed screen corner, updated
// each refresh from the current heading.
// Heading readout: a single label that names the direction the vehicle is
// CURRENTLY travelling (user-requested 2026-09-24) — heading North shows "N",
// South "S", etc., using the 8 compass directions. Not a rotating rose; just
// the current-travel label. kCompass8[round(heading/45)%8].
static lv_obj_t *compassLabel = nullptr;
static lv_obj_t *gVignette[4] = {nullptr, nullptr, nullptr, nullptr}; // edge-fade strips (top, bottom, left, right)
static lv_grad_dsc_t *gVignetteGrad[4] = {nullptr, nullptr, nullptr, nullptr};
static lv_obj_t *gTopBar = nullptr, *gBottomBar = nullptr;
// Board temperature readout (user-requested 2026-09-25): the device sits on a
// car windscreen and gets hot, so the ESP32-S3 die temperature is shown in the
// bottom-RIGHT corner (mirroring the heading letter bottom-left), colour-coded
// and with safety thresholds — see refreshDashboard() and kTempWarnC below.
static lv_obj_t *tempLabel = nullptr;
static int tempCx = 446, tempCy = 286; // bottom-right corner (mirror of the compass)
// ESP32-S3 die temperature thresholds. The on-chip sensor reads DIE temp, which
// idles ~50-60C and runs 70-80C under load even at room temperature, so the
// warning line is set well above that. Easy to tune after observing real
// readings on the windscreen in the sun.
static const float kTempWarnC = 80.0f; // amber + one-shot notice at/above this
static const float kTempCritC = 92.0f; // red blinking + repeating alarm + auto-dim to shed heat
static const char *kCompass8[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
static int northCx = 34, northCy = 34; // label center (screen px)
static uint32_t lastDrawnMapGeneration = 0xFFFFFFFFu;
static bool mapDimmed = false;
static int egoAnchorX = 240, egoAnchorY = 213;
static int gCanvasW = 480, gCanvasH = 320;
#ifdef VIETHUD_P4
// Native-resolution map layer (physical pixels) — see buildMapCanvas().
static int gMapPhysW = 800, gMapPhysH = 480, gMapAnchorPX = 400, gMapAnchorPY = 240;
#endif
// Landscape status bars: top (GNSS / street / clock / WiFi / settings) and the
// bottom one holding the heading letter + board temperature — both 20px text,
// both marked by the same 1px line.
static const int kTopBarH = 40;
static const int kBottomBarH = 40;
static int gMapSideS = 578, gMapCenter = 289; // oversized heading-up canvas: square side + its center (rotation pivot)
static int gRefMinDim = 160;                  // on-screen minDim framing the zoom (see buildMapCanvas)
static float gMapZoomScale = 2.0f; // default 2.0x (2026-09-27 user request); tap cycles 2.0 -> 2.5 -> 1.5
uint32_t g_mapDrawUs = 0, g_mapDrawCount = 0;

// --- Top bar ---
static lv_obj_t *gnssIcon, *gnssCaption;
static lv_obj_t *sunIcon, *clockLabel;
static lv_obj_t *gearIcon;
static lv_obj_t *wifiTopIcon; // shown only while WiFi is on
static lv_obj_t *streetNameBadge = nullptr;
static lv_obj_t *streetNameIcon = nullptr;
static lv_obj_t *streetNameLabel = nullptr;
static lv_obj_t *colDividerLine[2];

// --- Left column: speed + speed limit ---
// Audio icon/state label removed 2026-09-15 (user-requested) — never
// re-added; cfg.audioEnabled (AppConfig.h) is now the sign/camera/speed-
// limit alert-audio master toggle (Settings > Display), with no on-screen
// telltale, same as there being no buzzer output yet either.
static lv_obj_t *leftCol;
static lv_obj_t *speedLabel, *kmhCaption;
// Speed limit shown as an actual Vietnamese regulatory sign (QCVN 41:2019/
// BGTVT's P.127 "Tốc độ tối đa cho phép" — white circle, red ring, black
// number, no other markings), not a text readout — user-requested
// 2026-09-15. speedLimitSign is the ring/circle graphic (hidden when there's
// no valid limit — a real sign has no "unknown" state, so this dashboard
// doesn't invent one); speedLimitValueLabel is a SEPARATE sibling (not a
// child of the sign) positioned on top of it via align_to, so it can keep
// showing a plain "--" (same convention every other not-yet-available
// reading on this dashboard uses) even while the sign graphic itself is
// hidden.
static lv_obj_t *speedLimitSign, *speedLimitValueLabel;

// --- Middle column: sign/camera alert card ---
static lv_obj_t *midCol;
// Upcoming speed-limit-change banner (user-requested 2026-09-21, "canh bao
// gioi han toc do doan duong tiep theo, bao truoc khoang 100m") — one
// shared widget, built once, working unchanged in either orientation (see
// its build site in buildDashboard()). Own dedicated color (blue, a common
// real-world "informational" road-sign color), deliberately NOT red/amber,
// so it never reads as a safety alert the way the speeding overlay does —
// this is advance notice of a rule change ahead, not a danger warning.
// REMOVED 2026-09-24 (V2): aheadLimitLabel / cameraAheadLabel / trafficSignLabel
// were legacy floating banners pinned at the default (0,0) top-left corner.
// aheadLimitLabel and cameraAheadLabel had NO background styling, so they drew
// as faint unstyled text directly over the map (the "dòng text mờ góc trên
// trái" the user asked to remove). They duplicated the dedicated traffic alert
// card (trafficCard, winner-based W_CAMERA / W_AHEAD_LIMIT / W_RESIDENT / … /
// W_DANGER), which shows the same info with proper icons + distance + progress.
// The audio cues these blocks used to fire are preserved in refreshDashboard().

// Dedicated Traffic & Camera Alert Card (Minimalist HUD: Icon + Mini Speed Sign + Distance)
static lv_obj_t *trafficCard = nullptr;
static lv_obj_t *alertIconImg = nullptr;
static lv_obj_t *alertMiniSpeedSign = nullptr;
static lv_obj_t *alertMiniSpeedVal = nullptr;
static lv_obj_t *alertDistLabel = nullptr;
static lv_obj_t *alertProgressBar = nullptr;

static void buildTrafficCard(lv_obj_t *parent, int w, int h) {
    trafficCard = lv_obj_create(parent);
    lv_obj_set_size(trafficCard, w, h);
    lv_obj_align(trafficCard, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(trafficCard, lv_color_hex(0x0C1420), 0);
    lv_obj_set_style_bg_opa(trafficCard, LV_OPA_90, 0);
    lv_obj_set_style_border_color(trafficCard, lv_color_hex(0x28384C), 0);
    lv_obj_set_style_border_width(trafficCard, 2, 0);
    lv_obj_set_style_radius(trafficCard, h / 2, 0); // Sleek capsule shape
    lv_obj_set_style_pad_all(trafficCard, 0, 0);
    lv_obj_set_style_shadow_color(trafficCard, lv_color_hex(0x000000), 0);
    lv_obj_set_style_shadow_width(trafficCard, 12, 0);
    lv_obj_set_style_shadow_opa(trafficCard, LV_OPA_60, 0);
    lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_CLICKABLE);

    // Left Slot 1: Official 36x36 QCVN Sign Icon / Camera Icon
    alertIconImg = lv_image_create(trafficCard);
    lv_image_set_src(alertIconImg, &camera_icon);
    lv_obj_set_size(alertIconImg, 36, 36);
    lv_obj_align(alertIconImg, LV_ALIGN_LEFT_MID, 16, 0);
    lv_obj_clear_flag(alertIconImg, LV_OBJ_FLAG_CLICKABLE);

    // Left Slot 2: Mini P.127 Speed Limit Sign (Diameter 38px, Circular with Red Border)
    static const int kMiniSignDiam = 38;
    alertMiniSpeedSign = lv_obj_create(trafficCard);
    lv_obj_set_size(alertMiniSpeedSign, kMiniSignDiam, kMiniSignDiam);
    lv_obj_align(alertMiniSpeedSign, LV_ALIGN_LEFT_MID, 58, 0);
    lv_obj_set_style_radius(alertMiniSpeedSign, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(alertMiniSpeedSign, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(alertMiniSpeedSign, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(alertMiniSpeedSign, lv_color_hex(0xE60000), 0);
    lv_obj_set_style_border_width(alertMiniSpeedSign, 4, 0);
    lv_obj_set_style_pad_all(alertMiniSpeedSign, 0, 0);
    lv_obj_clear_flag(alertMiniSpeedSign, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(alertMiniSpeedSign, LV_OBJ_FLAG_CLICKABLE);

    alertMiniSpeedVal = lv_label_create(alertMiniSpeedSign);
    lv_obj_set_style_text_font(alertMiniSpeedVal, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(alertMiniSpeedVal, lv_color_black(), 0);
    lv_obj_align(alertMiniSpeedVal, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(alertMiniSpeedVal, "60");

    // Right Slot: Large countdown distance ("350 m", "120 m")
    alertDistLabel = lv_label_create(trafficCard);
    lv_obj_set_style_text_font(alertDistLabel, &lv_font_montserrat_36, 0);
    lv_obj_set_style_text_color(alertDistLabel, lv_color_white(), 0);
    lv_obj_align(alertDistLabel, LV_ALIGN_RIGHT_MID, -20, 0);
    lv_label_set_text(alertDistLabel, "");

    // Bottom edge: Thin progress bar
    alertProgressBar = lv_bar_create(trafficCard);
    lv_obj_set_size(alertProgressBar, w - 36, 3);
    lv_obj_align(alertProgressBar, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_bar_set_range(alertProgressBar, 0, 350);
    lv_bar_set_value(alertProgressBar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(alertProgressBar, lv_color_hex(0x182434), 0);
    lv_obj_set_style_bg_color(alertProgressBar, lv_color_hex(0xE0A020), LV_PART_INDICATOR);
    lv_obj_set_style_radius(alertProgressBar, 2, 0);
    lv_obj_set_style_radius(alertProgressBar, 2, LV_PART_INDICATOR);

    // Hidden by default
    lv_obj_add_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);
}

// --- Bottom info bar ---
static lv_obj_t *bottomInfoLabel;

// Full-screen speeding overlay (user-requested 2026-09-16, "neu vuot qua
// toc do toi da thi cung canh bao bang layer mau toan man hinh", then "qua
// toc do cung nhap nhay de tang su chu y") — its own dedicated color
// (0xB33DC6, distinct from the red/amber the sign/camera banners use) so
// speeding never reads as "you're about to hit something" or gets confused
// with a routine sign notice. Blinks at its own rate — see
// refreshDashboard()'s own comment on this overlay.
static lv_obj_t *speedingFlashOverlay;

// --- Small centred toast (map zoom level) ---
static lv_obj_t *wifiToastLabel;
static uint32_t wifiToastUntilMs = 0;

// Long-press-to-open-Settings feedback: a ring that sweeps closed over the
// hold duration, so a press registers visually right away instead of the
// screen doing nothing for a full second (spec section 17 discoverability).
static const int kHoldRingDiam = 48;
static lv_obj_t *holdRing;
static lv_anim_t holdRingAnim;

static void holdRingAnimCb(void *var, int32_t v) { lv_arc_set_value((lv_obj_t *)var, v); }

static void hideHoldRing() {
    lv_anim_delete(holdRing, holdRingAnimCb);
    lv_obj_add_flag(holdRing, LV_OBJ_FLAG_HIDDEN);
}

// Dashboard touch: a short tap cycles the map zoom; holding ~1s
// (HOLD_PRESS_MS, the ring animation's duration) opens Settings on release.
// The old 4s hold that toggled WiFi was removed 2026-09-26 (user request) —
// WiFi is switched only from Settings > WiFi now.
//
// A third tier (2s hold) and a double-tap gesture used to live here too,
// both toggling cfg.simpleUiMode (a radar-target-distance display mode) —
// removed 2026-09-21 alongside cfg.simpleUiMode itself when radar was taken
// out entirely (there's no more target distance for that mode to show).
static uint32_t pressStartMs = 0;
static bool longPressFired = false;    // past the 1s mark at least
// Hold ~2.5 s = toggle alert audio (2026-09-26, "cham man hinh 2-3s de tat
// hoac bat audio"). Fires WHILE still held (so the driver feels it happen and
// can let go); releasing afterwards then does nothing. Between 1 s and 2.5 s
// the ring re-sweeps in amber, and releasing in that window opens Settings.
static const uint32_t kAudioHoldMs = 2500;
static bool audioToggledThisPress = false;
static lv_obj_t *audioTopIcon = nullptr;
static bool lastAudioIconState = true;
static bool audioIconInit = false;
static void updateAudioTopIcon() {
    if (!audioTopIcon) return;
    if (audioIconInit && lastAudioIconState == cfg.audioEnabled) return;
    audioIconInit = true;
    lastAudioIconState = cfg.audioEnabled;
    lv_label_set_text(audioTopIcon, cfg.audioEnabled ? LV_SYMBOL_VOLUME_MAX : LV_SYMBOL_MUTE);
    lv_obj_set_style_text_color(audioTopIcon, cfg.audioEnabled ? lv_color_hex(pal().neutral) : lv_color_hex(0xFF5A4F), 0);
}

// Guards onDashReleasedOrLost's body from running more than once per
// physical press. LVGL can fire BOTH LV_EVENT_RELEASED and LV_EVENT_PRESS_LOST
// for what a user experiences as one clean tap (this board's touch
// controller has a known occasional stuck/glitch quirk — see
// AXS15231BTouch.cpp — that can plausibly trigger this).
static bool releaseHandledThisPress = false;

static void onDashPressed(lv_event_t *e) {
    lv_indev_t *indev = lv_event_get_indev(e);
    if (!indev) return;
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    lv_obj_set_pos(holdRing, p.x - kHoldRingDiam / 2, p.y - kHoldRingDiam / 2);
    lv_arc_set_value(holdRing, 0);
    lv_obj_clear_flag(holdRing, LV_OBJ_FLAG_HIDDEN);

    lv_anim_init(&holdRingAnim);
    lv_anim_set_var(&holdRingAnim, holdRing);
    lv_anim_set_exec_cb(&holdRingAnim, holdRingAnimCb);
    lv_anim_set_values(&holdRingAnim, 0, 100);
    lv_anim_set_time(&holdRingAnim, HOLD_PRESS_MS);
    lv_anim_start(&holdRingAnim);

    pressStartMs = millis();
    longPressFired = false;
    audioToggledThisPress = false;
    releaseHandledThisPress = false;
    lv_obj_set_style_arc_color(holdRing, lv_color_hex(0x4AA3FF), LV_PART_INDICATOR);
}

static void onDashLongPressed(lv_event_t *) {
    longPressFired = true;
    // Second stage: amber sweep towards the audio toggle.
    lv_anim_delete(holdRing, holdRingAnimCb);
    lv_obj_set_style_arc_color(holdRing, lv_color_hex(0xF0A020), LV_PART_INDICATOR);
    lv_arc_set_value(holdRing, 0);
    lv_anim_init(&holdRingAnim);
    lv_anim_set_var(&holdRingAnim, holdRing);
    lv_anim_set_exec_cb(&holdRingAnim, holdRingAnimCb);
    lv_anim_set_values(&holdRingAnim, 0, 100);
    lv_anim_set_time(&holdRingAnim, kAudioHoldMs - HOLD_PRESS_MS);
    lv_anim_start(&holdRingAnim);
}

static void onDashLongPressedRepeat(lv_event_t *) {
    if (audioToggledThisPress || millis() - pressStartMs < kAudioHoldMs) return;
    audioToggledThisPress = true;
    hideHoldRing();
    cfg.audioEnabled = !cfg.audioEnabled;
    saveConfigToNVS(cfg);
    settingsSyncSwitches();
    updateAudioTopIcon();
    if (cfg.audioEnabled) audioPlayBeep(1); // audible confirmation it's back on
    Serial.printf("[ui] alert audio %s via 2.5 s hold\n", cfg.audioEnabled ? "ON" : "OFF");
    if (wifiToastLabel) {
        lv_label_set_text(wifiToastLabel, cfg.audioEnabled ? LV_SYMBOL_VOLUME_MAX "  Sound: ON"
                                                           : LV_SYMBOL_MUTE "  Sound: OFF");
        lv_obj_clear_flag(wifiToastLabel, LV_OBJ_FLAG_HIDDEN);
        wifiToastUntilMs = millis() + 1500;
    }
}

static void onDashReleasedOrLost(lv_event_t *) {
    hideHoldRing();
    if (releaseHandledThisPress) return;
    releaseHandledThisPress = true;
    if (audioToggledThisPress) return; // the hold already did its thing
    if (longPressFired) {
        lv_screen_load(settingsScreen);
        return;
    }
    // Short tap: Cycle map zoom level (1.5x -> 2.0x -> 2.5x -> 1.5x)
    if (gMapZoomScale < 1.7f) {
        gMapZoomScale = 2.0f;
    } else if (gMapZoomScale < 2.3f) {
        gMapZoomScale = 2.5f;
    } else {
        gMapZoomScale = 1.5f;
    }
    // Drive the map zoom (published zoomRadiusM). Force an
    // immediate redraw by invalidating the last-drawn generation.
    mapRendererSetZoomMultiplier(gMapZoomScale);
    lastDrawnMapGeneration = 0;
    char zBuf[32];
    snprintf(zBuf, sizeof(zBuf), "Zoom: %.1fx", (double)gMapZoomScale);
    if (wifiToastLabel) {
        lv_label_set_text(wifiToastLabel, zBuf);
        lv_obj_clear_flag(wifiToastLabel, LV_OBJ_FLAG_HIDDEN);
        wifiToastUntilMs = millis() + 1200;
    }
    lastDrawnMapGeneration = 0; // Trigger redraw with new zoom level immediately
}

// Plain, non-interactive container — used for the top-bar row and the three
// columns. Transparent/borderless so it's invisible except for its children.

// ---------------------------------------------------------------------
// Full-Screen Map Canvas (vector roads + markers + trail)
// ---------------------------------------------------------------------


struct RoadClassStyle {
    lv_color_t color;
    int32_t width;
};
static RoadClassStyle mapClassStyle(uint8_t roadClass) {
    // Neutral, understated palette (user-requested "màu trung tính ... layer
    // ẩn"): soft greys that read on the dark map without shouting; the road
    // you're on keeps a muted cyan accent so it's still findable at a glance.
    switch (roadClass) {
        case kMapLineCurrentRoad: return {lv_color_hex(pal().roadCurrent), 5}; // current road (cyan accent)
        case 1: return {lv_color_hex(pal().roadMajor), 4};  // major
        case 2: return {lv_color_hex(pal().roadMain), 3};   // main
        default: return {lv_color_hex(pal().roadSmall), 2}; // small/other
    }
}

static lv_color_t mapMarkerColor(uint8_t kind) {
    switch (kind) {
        case MAP_MARKER_CAMERA: return lv_color_hex(0xE0A020);
        case MAP_MARKER_RESIDENT_AREA: return lv_color_hex(0x2080C0);
        case MAP_MARKER_NO_OVERTAKING: return lv_color_hex(0xD04020);
        case MAP_MARKER_TRAFFIC_LIGHT: return lv_color_hex(0x209060);
        case MAP_MARKER_TOLL_BOOTH: return lv_color_hex(0x7050B0);
        default: return lv_color_hex(0x90A4B8);
    }
}

static void buildMapCanvas(lv_obj_t *parent, int w, int h) {
    gCanvasW = w;
    gCanvasH = h;

    // On-screen ego anchor (where the car icon sits) — keep the existing
    // placement so the driver's viewpoint is unchanged.
    int screenAnchorX = w / 2;
    int screenAnchorY = (h > w) ? 200 : (h / 2);
    egoAnchorX = screenAnchorX;
    egoAnchorY = screenAnchorY;

    // Heading-up map (2026-09-24): the map canvas is drawn NORTH-UP and rotated
    // as a whole (LVGL image rotation) so travel points at 12 o'clock. For the
    // rotation to never leave a screen corner blank, the canvas is an OVERSIZED
    // SQUARE whose inscribed circle reaches every screen corner from the ego
    // anchor. Its center is the ego (rotation pivot); it's positioned so that
    // center lands on the on-screen anchor.
    auto cornerDist = [&](int cx, int cy) {
        float best = 0;
        int xs[2] = {0, w}, ys[2] = {0, h};
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) {
                float dx = xs[i] - cx, dy = ys[j] - cy;
                float d = sqrtf(dx * dx + dy * dy);
                if (d > best) best = d;
            }
        return best;
    };
    int maxR = (int)ceilf(cornerDist(screenAnchorX, screenAnchorY)) + 4;
    int S = maxR * 2;
    if (S & 1) S++; // even, so center is exact
    gMapSideS = S;
    gMapCenter = S / 2;
    // Reference minDim that frames the zoom to the VISIBLE screen (all four
    // margins from the on-screen anchor), independent of the oversized canvas.
    int refMin = screenAnchorX;
    if (screenAnchorY < refMin) refMin = screenAnchorY;
    if (w - screenAnchorX < refMin) refMin = w - screenAnchorX;
    if (h - screenAnchorY < refMin) refMin = h - screenAnchorY;
    if (refMin < 1) refMin = 1;
    gRefMinDim = refMin;

#ifdef VIETHUD_P4
    // P4: the map is its own native-resolution layer (800x480): a screen-
    // sized canvas that is NOT rotated as a bitmap — updateMapCanvas() bakes
    // the heading-up rotation into the vector coordinates instead, so roads
    // stay crisp antialiased lines at any angle (the S3's nearest-neighbour
    // bitmap rotation of a square canvas looked jagged). The MapRenderer
    // square geometry (S, refMin) is computed in physical pixels too.
    {
        gMapPhysW = displayPhysWidth();
        gMapPhysH = displayPhysHeight();
        gMapAnchorPX = vhS(screenAnchorX);
        gMapAnchorPY = vhS(screenAnchorY);
        egoAnchorX = gMapAnchorPX; // edge fade works in physical screen space here
        egoAnchorY = gMapAnchorPY;
        float best = 0;
        int xs[2] = {0, gMapPhysW}, ys[2] = {0, gMapPhysH};
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) {
                float dx = xs[i] - gMapAnchorPX, dy = ys[j] - gMapAnchorPY;
                best = fmaxf(best, sqrtf(dx * dx + dy * dy));
            }
        S = ((int)ceilf(best) + 4) * 2;
        gMapSideS = S;
        gMapCenter = S / 2;
        int rm = gMapAnchorPX;
        rm = rm < gMapAnchorPY ? rm : gMapAnchorPY;
        rm = rm < gMapPhysW - gMapAnchorPX ? rm : gMapPhysW - gMapAnchorPX;
        rm = rm < gMapPhysH - gMapAnchorPY ? rm : gMapPhysH - gMapAnchorPY;
        gRefMinDim = rm > 1 ? rm : 1;
        size_t bytes = (size_t)gMapPhysW * gMapPhysH * sizeof(uint16_t);
        mapCanvasBuf = (uint16_t *)heap_caps_aligned_alloc(64, bytes, MALLOC_CAP_SPIRAM);
        mapCanvas = lv_canvas_create(parent);
        lv_canvas_set_buffer(mapCanvas, mapCanvasBuf, gMapPhysW, gMapPhysH, LV_COLOR_FORMAT_RGB565);
        (lv_obj_set_pos)(mapCanvas, 0, 0);
        (lv_obj_set_size)(mapCanvas, gMapPhysW, gMapPhysH);
        lv_canvas_fill_bg(mapCanvas, lv_color_hex(0x06080C), LV_OPA_COVER);
        lv_obj_set_style_opa(mapCanvas, LV_OPA_COVER, 0);
        lv_obj_clear_flag(mapCanvas, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(mapCanvas, LV_OBJ_FLAG_CLICKABLE);
    }
#else
    size_t bufBytes = (size_t)S * (size_t)S * sizeof(uint16_t);
    mapCanvasBuf = (uint16_t *)heap_caps_malloc(bufBytes, MALLOC_CAP_SPIRAM);
    if (!mapCanvasBuf) {
        Serial.println("[ui] WARN: PSRAM allocation for map canvas failed, falling back to internal RAM");
        mapCanvasBuf = (uint16_t *)malloc(bufBytes);
    }
    mapCanvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(mapCanvas, mapCanvasBuf, S, S, LV_COLOR_FORMAT_RGB565);
    lv_obj_set_pos(mapCanvas, screenAnchorX - S / 2, screenAnchorY - S / 2);
    lv_obj_set_size(mapCanvas, S, S);
    lv_canvas_fill_bg(mapCanvas, lv_color_hex(0x06080C), LV_OPA_COVER);
    lv_obj_set_style_opa(mapCanvas, LV_OPA_COVER, 0); // opaque: avoids blending the large map canvas over the bg every redraw (perf)
    lv_obj_clear_flag(mapCanvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(mapCanvas, LV_OBJ_FLAG_CLICKABLE);
    // Rotate the whole canvas about the ego (its center) — set each redraw in
    // updateMapCanvas(); pivot is fixed here.
    lv_image_set_pivot(mapCanvas, S / 2, S / 2);
    lv_image_set_antialias(mapCanvas, false); // nearest-neighbor: far cheaper per rotated frame on the ESP32
#endif

    mapRendererSetGeometry(S, S / 2, S / 2, gRefMinDim);
    mapRendererSetZoomMultiplier(gMapZoomScale);

    int anchorX = screenAnchorX;
    int anchorY = screenAnchorY;

    // Outer navigation pulse ring
    egoHalo = lv_obj_create(parent);
    lv_obj_set_size(egoHalo, 38, 38);
    lv_obj_set_pos(egoHalo, anchorX - 19, anchorY - 19);
    lv_obj_set_style_radius(egoHalo, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(egoHalo, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_bg_opa(egoHalo, LV_OPA_20, 0);
    lv_obj_set_style_border_color(egoHalo, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_border_width(egoHalo, 1, 0);
    lv_obj_set_style_border_opa(egoHalo, LV_OPA_50, 0);
    lv_obj_clear_flag(egoHalo, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(egoHalo, LV_OBJ_FLAG_SCROLLABLE);

    // Modern 3D Navigation Chevron (Tip, Right wing, Center notch, Left wing, Tip)
    static lv_point_precise_t arrowPts[6];
    arrowPts[0] = {(lv_value_precise_t)anchorX, (lv_value_precise_t)(anchorY - 15)};
    arrowPts[1] = {(lv_value_precise_t)(anchorX + 11), (lv_value_precise_t)(anchorY + 11)};
    arrowPts[2] = {(lv_value_precise_t)anchorX, (lv_value_precise_t)(anchorY + 5)};
    arrowPts[3] = {(lv_value_precise_t)(anchorX - 11), (lv_value_precise_t)(anchorY + 11)};
    arrowPts[4] = arrowPts[0];
    egoArrow = lv_line_create(parent);
    lv_line_set_points(egoArrow, arrowPts, 5);
    lv_obj_set_style_line_width(egoArrow, 3, 0);
    lv_obj_set_style_line_color(egoArrow, lv_color_hex(0x00E5FF), 0);
    lv_obj_set_style_line_rounded(egoArrow, true, 0);
    lv_obj_clear_flag(egoArrow, LV_OBJ_FLAG_CLICKABLE);

    // (Edge fade: drawn into the map itself — see updateMapCanvas(). The old
    // overlay strips here couldn't blend cleanly on RGB565.)

    // --- Heading readout: one label in the bottom-left corner showing the
    // current travel direction as a compass letter (N/NE/E/SE/S/SW/W/NW),
    // updated from GNSS heading in refreshDashboard().
    northCx = 34;
    northCy = gCanvasH - kBottomBarH / 2; // centred in the bottom bar (line at gCanvasH - kBottomBarH)
    compassLabel = lv_label_create(parent);
    lv_label_set_text(compassLabel, "N");
    lv_obj_set_style_text_font(compassLabel, &lv_font_montserrat_20, 0); // bigger heading letter (2026-09-25)
    lv_obj_set_style_text_color(compassLabel, lv_color_hex(0x00E5FF), 0); // cyan accent (matches the ego arrow)
    lv_obj_clear_flag(compassLabel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_pos(compassLabel, northCx - 6, northCy - 11);

    // Board temperature readout, bottom-right corner (mirror of the compass).
    tempCx = gCanvasW - 34;
    tempCy = gCanvasH - kBottomBarH / 2;
    tempLabel = lv_label_create(parent);
    lv_label_set_text(tempLabel, "--\xC2\xB0" "C"); // "--°C" until the first reading (°=U+00B0, UTF-8 C2 B0)
    lv_obj_set_style_text_font(tempLabel, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(tempLabel, lv_color_hex(0x93A0AE), 0); // neutral until a reading colours it
    lv_obj_clear_flag(tempLabel, LV_OBJ_FLAG_CLICKABLE);
    // Fixed width, right-aligned so the value stays anchored in the corner
    // whether it's "40°C" or "100°C" (no per-reading repositioning needed).
    lv_obj_set_width(tempLabel, 72);
    lv_obj_set_style_text_align(tempLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(tempLabel, gCanvasW - 8 - 72, tempCy - 11); // right edge ~8px from the screen edge
}

#ifdef VIETHUD_P4
volatile bool gMapPaused = false; // flicker diagnostic phase C (main_viethud.cpp 'Z')
#endif
static void updateMapCanvas() {
    if (!mapCanvas) return;
#ifdef VIETHUD_P4
    if (gMapPaused) return;
#endif
    esp_task_wdt_reset();

    static MapViewSnapshot v;
    v = mapViewSnapshot();

    // NOTE (2026-09-24 V2): the per-tick lv_obj_set_style_opa(mapCanvas,...) that
    // used to be here was the hidden cause of a full-screen flush EVERY frame.
    // A style write invalidates the object (LVGL doesn't diff it), and this
    // object is the full-screen rotated map image — so every 150ms tick the
    // whole map recomposited + pushed the whole 307KB frame (~33ms), even when
    // nothing had moved. The opacity is set once at creation (buildMapCanvas);
    // it never changes, so setting it here again was pure invalidation cost.

    // Track last known coordinates or fallback to Vietnam center (21.0278, 105.8342)
    static float sLastMapLat = 21.0278f;
    static float sLastMapLon = 105.8342f;
    static bool sMapDrawnOnce = false;

    GnssSnapshot gnss = gnssSnapshot();
    if (gnss.fix && gnss.latDeg > 1.0f) {
        sLastMapLat = (float)gnss.latDeg;
        sLastMapLon = (float)gnss.lonDeg;
    }

    // No GPS fix yet: clear the canvas to the map background once (vector-only
    // map — there is nothing to draw until a position arrives), then wait.
    if (!v.valid) {
        if (!sMapDrawnOnce || gThemeRepaintMap) {
            sMapDrawnOnce = true;
            gThemeRepaintMap = false;
            lv_canvas_fill_bg(mapCanvas, lv_color_hex(pal().bg), LV_OPA_COVER);
            lv_image_set_rotation(mapCanvas, 0);
        }
        return;
    }

    // Only redraw when map generation actually changed (movement >= 3m or turn >= 3 deg)
    if (v.generation == lastDrawnMapGeneration && !gThemeRepaintMap) return;
    gThemeRepaintMap = false;
    lastDrawnMapGeneration = v.generation;
    sMapDrawnOnce = true;

    uint32_t drawStartUs = micros();

    // Map centre = the snapped ego point the vector layer was projected about.
    float centerLat = (v.egoLatDeg > 1.0f) ? v.egoLatDeg : sLastMapLat;
    float centerLon = (v.egoLonDeg > 1.0f) ? v.egoLonDeg : sLastMapLon;
    if (v.egoLatDeg > 1.0f) { sLastMapLat = v.egoLatDeg; sLastMapLon = v.egoLonDeg; }
    float radiusM = (v.zoomRadiusM > 10.0f) ? v.zoomRadiusM : 300.0f;
    float pxPerM = (float)gRefMinDim / radiusM;

#ifdef VIETHUD_P4
    int32_t rot = 0;
    {
        // Heading-up rotation baked into the geometry: MapRenderer projects
        // about the centre of its SxS square; each point is rotated about that
        // centre and translated onto the ego anchor of the screen-sized
        // canvas. Same maths (and sign) as the S3 bitmap rotation it replaces.
        lv_canvas_fill_bg(mapCanvas, lv_color_hex(pal().bg), LV_OPA_COVER);
        lv_layer_t layer;
        lv_canvas_init_layer(mapCanvas, &layer);
        esp_task_wdt_reset();
        if (cfg.mapHeadingUp) {
            rot = (int32_t)lroundf(-v.headingUpDeg * 10.0f);
            rot %= 3600;
            if (rot < 0) rot += 3600;
        }
        const float rr = rot * (float)M_PI / 1800.0f;
        const float cR = cosf(rr), sR = sinf(rr);
        const float cc = (float)gMapCenter;
        const float ax0 = (float)gMapAnchorPX, ay0 = (float)gMapAnchorPY;
        auto toScr = [&](float x, float y, float *sx, float *sy) {
            float dx = x - cc, dy = y - cc;
            *sx = ax0 + dx * cR - dy * sR;
            *sy = ay0 + dx * sR + dy * cR;
        };
        const float W = (float)gMapPhysW, H = (float)gMapPhysH;
        auto edgeOpaS = [&](float sx, float sy) -> lv_opa_t {
            float fx = fminf(sx, W - sx) / 150.0f;
            float fy = fminf(sy, H - sy) / 120.0f;
            float f = fminf(fx, fy);
            if (f <= 0.0f) return 0;
            if (f >= 1.0f) return LV_OPA_COVER;
            return (lv_opa_t)(f * f * (3.0f - 2.0f * f) * 255.0f);
        };
        lv_draw_line_dsc_t ld;
        auto seg = [&](float x1, float y1, float x2, float y2, lv_color_t col, int32_t w, lv_opa_t opa) {
            lv_draw_line_dsc_init(&ld);
            ld.color = col;
            ld.width = w;
            ld.opa = opa;
            // Round caps only on opaque strokes: overlapping translucent caps
            // (edge-faded pieces) would show as beads.
            ld.round_start = ld.round_end = opa >= LV_OPA_MAX ? 1 : 0;
            ld.p1.x = (lv_value_precise_t)x1;
            ld.p1.y = (lv_value_precise_t)y1;
            ld.p2.x = (lv_value_precise_t)x2;
            ld.p2.y = (lv_value_precise_t)y2;
            lv_draw_line(&layer, &ld);
        };
        // A line in screen space, faded toward the screen edges; drawn in one
        // stroke when it lies entirely in the fully-opaque interior.
        auto fadedLineS = [&](float x1, float y1, float x2, float y2, lv_color_t col, int32_t w, lv_opa_t base) {
            if ((x1 < 0 && x2 < 0) || (y1 < 0 && y2 < 0) || (x1 > W && x2 > W) || (y1 > H && y2 > H)) return;
            if (edgeOpaS(x1, y1) == LV_OPA_COVER && edgeOpaS(x2, y2) == LV_OPA_COVER) {
                seg(x1, y1, x2, y2, col, w, base);
                return;
            }
            float len = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            int n = (int)ceilf(len / 24.0f);
            if (n < 1) n = 1;
            for (int k = 0; k < n; k++) {
                float t0 = (float)k / n, t1 = (float)(k + 1) / n;
                float ax = x1 + (x2 - x1) * t0, ay = y1 + (y2 - y1) * t0;
                float bx = x1 + (x2 - x1) * t1, by = y1 + (y2 - y1) * t1;
                lv_opa_t o = edgeOpaS((ax + bx) * 0.5f, (ay + by) * 0.5f);
                if (o < 8) continue;
                seg(ax, ay, bx, by, col, w, (lv_opa_t)((o * base) / 255));
            }
        };
        // Pass order: small roads, then casing + fill for main, major, and the
        // road we're on last, so junctions layer the way a printed map reads.
        // Within a class, ALL casings first, then all fills — casing drawn per
        // segment would notch the previous segment's fill (dashed look).
        const lv_color_t casing = lv_color_mix(lv_color_black(), lv_color_hex(pal().bg), 150);
        for (int pass = 0; pass < 4; pass++) {
            for (int layerIdx = (pass > 0 ? 0 : 1); layerIdx < 2; layerIdx++) {
                for (int i = 0; i < v.lineCount; i++) {
                    const MapLine &ln = v.lines[i];
                    const int c = ln.roadClass;
                    const bool cur = c == kMapLineCurrentRoad;
                    const int rank = cur ? 3 : (c == 1 ? 2 : (c == 2 ? 1 : 0));
                    if (rank != pass) continue;
                    RoadClassStyle st = mapClassStyle(ln.roadClass);
                    int32_t w = st.width * 3 / 2 + (cur ? 2 : 0);
                    float x1, y1, x2, y2;
                    toScr(ln.x1, ln.y1, &x1, &y1);
                    toScr(ln.x2, ln.y2, &x2, &y2);
                    if (layerIdx == 0) fadedLineS(x1, y1, x2, y2, casing, w + 4, LV_OPA_COVER);
                    else fadedLineS(x1, y1, x2, y2, st.color, w, LV_OPA_COVER);
                }
            }
        }
        if (cfg.showVehicleTrail) {
            for (int i = 1; i < v.trailCount; i++) {
                int denom = v.trailCount > 1 ? v.trailCount - 1 : 1;
                float x1, y1, x2, y2;
                toScr(v.trailX[i - 1], v.trailY[i - 1], &x1, &y1);
                toScr(v.trailX[i], v.trailY[i], &x2, &y2);
                fadedLineS(x1, y1, x2, y2, lv_color_hex(ACCENT_COLOR), 4, (lv_opa_t)(60 + (195 * i) / denom));
            }
        }
        lv_draw_rect_dsc_t dot;
        for (int i = 0; i < v.markerCount; i++) {
            const MapMarker &m = v.markers[i];
            float sx, sy;
            toScr(m.x, m.y, &sx, &sy);
            lv_opa_t o = edgeOpaS(sx, sy);
            if (o < 8) continue;
            lv_draw_rect_dsc_init(&dot);
            dot.bg_color = mapMarkerColor(m.kind);
            dot.bg_opa = o;
            dot.radius = LV_RADIUS_CIRCLE;
            dot.border_color = lv_color_white();
            dot.border_width = 2;
            dot.border_opa = (lv_opa_t)((o * 200) / 255);
            lv_area_t area = {(int32_t)sx - 8, (int32_t)sy - 8, (int32_t)sx + 8, (int32_t)sy + 8};
            lv_draw_rect(&layer, &dot, &area);
        }
        lv_canvas_finish_layer(mapCanvas, &layer);
        esp_task_wdt_reset();
    }
#else
    lv_canvas_fill_bg(mapCanvas, lv_color_hex(pal().bg), LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(mapCanvas, &layer);

    esp_task_wdt_reset();

    // Heading-up rotation for this frame (applied to the canvas below); needed
    // first so each road piece's on-screen position is known for the edge fade.
    int32_t rot = 0;
    if (cfg.mapHeadingUp) {
        rot = (int32_t)lroundf(-v.headingUpDeg * 10.0f);
        rot %= 3600;
        if (rot < 0) rot += 3600;
    }
    // Edge fade (2026-09-27, "ban do tao gradient mo dan ve ben trai, ben phai
    // khoang 100px moi ben"): roads fade out over 100 px at the left/right screen
    // edges (80 px top/bottom). Done per road piece in the canvas, NOT with
    // overlay strips: an overlay in the map's own background colour can't blend
    // cleanly on RGB565 (every partial opacity lands one shade darker, leaving a
    // flat band with a hard edge) — and it cost two full-height blends a frame.
    const float rr = rot * (float)M_PI / 1800.0f;
    const float cR = cosf(rr), sR = sinf(rr);
    const float cc = (float)gMapCenter;
    auto edgeOpa = [&](float x, float y) -> lv_opa_t {
        float dx = x - cc, dy = y - cc;
        float sx = egoAnchorX + dx * cR - dy * sR;
        float sy = egoAnchorY + dx * sR + dy * cR;
        float fx = fminf(sx, gCanvasW - sx) / 100.0f;
        float fy = fminf(sy, gCanvasH - sy) / 80.0f;
        float f = fminf(fx, fy);
        if (f <= 0.0f) return 0;
        if (f >= 1.0f) return LV_OPA_COVER;
        return (lv_opa_t)(f * f * (3.0f - 2.0f * f) * 255.0f); // smoothstep
    };
    // Draw a line as pieces of <= kPiecePx, each with the fade opacity at its middle.
    const float kPiecePx = 16.0f;
    lv_draw_line_dsc_t lineDsc;
    auto fadedLine = [&](float x1, float y1, float x2, float y2, lv_color_t col, int32_t width, lv_opa_t baseOpa) {
        float len = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        int n = (int)ceilf(len / kPiecePx);
        if (n < 1) n = 1;
        for (int k = 0; k < n; k++) {
            float t0 = (float)k / n, t1 = (float)(k + 1) / n;
            float ax = x1 + (x2 - x1) * t0, ay = y1 + (y2 - y1) * t0;
            float bx = x1 + (x2 - x1) * t1, by = y1 + (y2 - y1) * t1;
            lv_opa_t o = edgeOpa((ax + bx) * 0.5f, (ay + by) * 0.5f);
            if (o < 8) continue;
            lv_draw_line_dsc_init(&lineDsc);
            lineDsc.color = col;
            lineDsc.width = width;
            lineDsc.opa = (lv_opa_t)((o * baseOpa) / 255);
            // Rounded ends only when opaque: overlapping round caps of two
            // translucent pieces would double-blend into darker dots.
            lineDsc.round_start = lineDsc.round_end = (lineDsc.opa >= LV_OPA_MAX) ? 1 : 0;
            lineDsc.p1.x = (lv_value_precise_t)ax;
            lineDsc.p1.y = (lv_value_precise_t)ay;
            lineDsc.p2.x = (lv_value_precise_t)bx;
            lineDsc.p2.y = (lv_value_precise_t)by;
            lv_draw_line(&layer, &lineDsc);
        }
    };

    esp_task_wdt_reset();

    // Pass 1: Vector Road Lines
    for (int i = 0; i < v.lineCount; i++) {
        const MapLine &ln = v.lines[i];
        RoadClassStyle style = mapClassStyle(ln.roadClass);
        fadedLine(ln.x1, ln.y1, ln.x2, ln.y2, style.color, style.width, LV_OPA_COVER);
    }

    // Pass 2: Breadcrumb trail (Polyline track)
    if (cfg.showVehicleTrail) {
        for (int i = 1; i < v.trailCount; i++) {
            int denom = v.trailCount > 1 ? v.trailCount - 1 : 1;
            fadedLine(v.trailX[i - 1], v.trailY[i - 1], v.trailX[i], v.trailY[i], lv_color_hex(ACCENT_COLOR), 3,
                      (lv_opa_t)(60 + (195 * i) / denom));
        }
    }

    // Pass 3: Markers (cameras and traffic signs)
    lv_draw_rect_dsc_t dotDsc;
    for (int i = 0; i < v.markerCount; i++) {
        const MapMarker &m = v.markers[i];
        lv_opa_t o = edgeOpa(m.x, m.y);
        if (o < 8) continue;
        lv_draw_rect_dsc_init(&dotDsc);
        dotDsc.bg_color = mapMarkerColor(m.kind);
        dotDsc.bg_opa = o;
        dotDsc.radius = LV_RADIUS_CIRCLE;
        lv_area_t area = {m.x - 5, m.y - 5, m.x + 5, m.y + 5};
        lv_draw_rect(&layer, &dotDsc, &area);
    }

    lv_canvas_finish_layer(mapCanvas, &layer);

    // Heading-up: rotate the whole north-up canvas by -heading so travel points
    // at 12 o'clock. LVGL rotates about the pivot set in buildMapCanvas (the
    // ego = canvas center), which lands on the on-screen anchor, so the car
    // stays fixed and the map spins/translates beneath it. Angle is 0.1° units,
    // clockwise-positive; -heading (mod 360) makes the travel direction up.
    // cfg.mapHeadingUp off = north-up (rotation 0), the simpler/proven mode.
    lv_image_set_rotation(mapCanvas, rot); // computed above (also used by the edge fade)
#endif
    esp_task_wdt_reset();

    // Periodic map-render diagnostics (every ~3s) so the map pipeline is
    // visible over the serial monitor when the screen shows nothing — added
    // 2026-09-24 after a "maps don't display" report.
    {
        static uint32_t sLastMapDbgMs = 0;
        uint32_t nowDbg = millis();
        if (nowDbg - sLastMapDbgMs > 3000) {
            sLastMapDbgMs = nowDbg;
            Serial.printf("[mapui] lines=%d markers=%d "
                          "headingUp=%d rot=%.0f center=%.5f,%.5f pxPerM=%.3f bufOK=%d\n",
                          v.lineCount, v.markerCount, cfg.mapHeadingUp,
                          (double)v.headingUpDeg, (double)centerLat, (double)centerLon, (double)pxPerM,
                          mapCanvasBuf != nullptr);
        }
    }

    g_mapDrawUs += micros() - drawStartUs;
    g_mapDrawCount++;
}

static lv_obj_t *makePane(lv_obj_t *parent, int x, int y, int w, int h) {
    lv_obj_t *o = lv_obj_create(parent);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_set_style_radius(o, 0, 0);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
    return o;
}

static lv_obj_t *makeIcon(lv_obj_t *parent, const lv_image_dsc_t *src) {
    lv_obj_t *img = lv_image_create(parent);
    lv_image_set_src(img, src);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
    return img;
}

// ---------------------------------------------------------------------
// Landscape content (rotation 1/3, 480x320) — everything between the shared
// dashRoot setup and the shared full-screen overlays in buildDashboard()
// below. Rewritten from scratch 2026-09-22 while chasing a real-hardware
// landscape rendering-corruption report (bottom bar text appearing
// vertically, a large isolated glyph) — the original arithmetic checked out
// on paper and no root cause was confirmed via LVGL logs, so rather than
// keep guessing this function was redone plain, with every dynamic-text
// label given an explicit width + clipped long-mode (see bottomInfoLabel
// below) so unconstrained content-hug sizing can't itself be a factor,
// whatever the real cause turns out to be. Same visual result as before:
// [ speed + speed-limit sign | sign/camera alert card ], top status bar,
// one-line bottom info bar. Still plain lv_obj_create/lv_obj_set_pos/
// lv_obj_set_size throughout (via makePane()) — no flex/grid anywhere in
// this file, deliberately, so there's no auto-sizing container that could
// ever collapse a child to near-zero.
// ---------------------------------------------------------------------
static void buildDashboardLandscape(lv_obj_t *scr) {
    const int scrW = LS_SCR_W, scrH = LS_SCR_H;
    const int topH = kTopBarH;

    // ---------------- Top status bar (Full width 480, Glassmorphism) ----------------
    lv_obj_t *topBar = makePane(scr, 0, 0, scrW, topH);
    gTopBar = topBar;
    lv_obj_set_style_bg_opa(topBar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(topBar, lv_color_hex(0x182232), 0);
    lv_obj_set_style_border_width(topBar, 1, 0);
    lv_obj_set_style_border_side(topBar, LV_BORDER_SIDE_BOTTOM, 0);

    // Left: GNSS status
    gnssIcon = lv_label_create(topBar);
    lv_obj_set_style_text_font(gnssIcon, &lv_font_montserrat_20, 0);
    lv_label_set_text(gnssIcon, LV_SYMBOL_GPS);
    lv_obj_align(gnssIcon, LV_ALIGN_LEFT_MID, 12, 0);

    gnssCaption = lv_label_create(topBar);
    lv_obj_set_style_text_font(gnssCaption, &lv_font_vn_20, 0);
    lv_label_set_text(gnssCaption, "--");
    lv_obj_align_to(gnssCaption, gnssIcon, LV_ALIGN_OUT_RIGHT_MID, 6, 0);

    // Right corner: clock only. (Settings gear and sun/moon icons removed from
    // the landscape bar 2026-09-26 per user; the objects still exist, hidden,
    // because the theme and day/night code update them.)
    clockLabel = lv_label_create(topBar);
    lv_obj_set_style_text_font(clockLabel, &lv_font_vn_20, 0);
    lv_label_set_text(clockLabel, "--:--");
    lv_obj_align(clockLabel, LV_ALIGN_RIGHT_MID, -12, 0); // style-based align: stays flush right as the text changes
    // Alert-audio status, left of the clock (fixed offset: the clock is right-
    // aligned and "HH:MM" is ~52 px in vn_20).
    audioTopIcon = lv_label_create(topBar);
    lv_obj_set_style_text_font(audioTopIcon, &lv_font_montserrat_20, 0);
    lv_obj_align(audioTopIcon, LV_ALIGN_RIGHT_MID, -76, 0);
    updateAudioTopIcon();

    gearIcon = lv_label_create(topBar);
    lv_obj_add_flag(gearIcon, LV_OBJ_FLAG_HIDDEN);
    sunIcon = makeIcon(topBar, &sun_icon);
    lv_obj_add_flag(sunIcon, LV_OBJ_FLAG_HIDDEN);

    // Center: Street Name Badge (Glassmorphism Pill)
    streetNameBadge = lv_obj_create(topBar);
    lv_obj_set_size(streetNameBadge, 264, 30);
    lv_obj_align(streetNameBadge, LV_ALIGN_CENTER, 0, 0); // centred; clears the GNSS block (left) and audio icon + clock (right)
    lv_obj_set_style_bg_color(streetNameBadge, lv_color_hex(0x0C1522), 0);
    lv_obj_set_style_bg_opa(streetNameBadge, LV_OPA_80, 0);
    lv_obj_set_style_border_color(streetNameBadge, lv_color_hex(0x1F314A), 0);
    lv_obj_set_style_border_width(streetNameBadge, 1, 0);
    lv_obj_set_style_radius(streetNameBadge, 15, 0);
    lv_obj_set_style_pad_all(streetNameBadge, 0, 0);
    lv_obj_clear_flag(streetNameBadge, LV_OBJ_FLAG_SCROLLABLE);

    streetNameIcon = lv_label_create(streetNameBadge);
    lv_obj_set_style_text_font(streetNameIcon, &lv_font_montserrat_20, 0);
    lv_label_set_text(streetNameIcon, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(streetNameIcon, lv_color_hex(0x00E5FF), 0);
    lv_obj_align(streetNameIcon, LV_ALIGN_LEFT_MID, 8, 0);

    streetNameLabel = lv_label_create(streetNameBadge);
    lv_obj_set_style_text_font(streetNameLabel, &lv_font_vn_20, 0);
    lv_obj_set_style_text_color(streetNameLabel, lv_color_hex(0xF0F4F8), 0);
    lv_label_set_long_mode(streetNameLabel, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(streetNameLabel, 224);
    lv_obj_align(streetNameLabel, LV_ALIGN_LEFT_MID, 30, 0);
    lv_label_set_text(streetNameLabel, "");
    lv_obj_add_flag(streetNameBadge, LV_OBJ_FLAG_HIDDEN);

    // Theme placeholders
    static lv_point_precise_t divPts[2][2];
    for (int i = 0; i < 2; i++) {
        colDividerLine[i] = lv_line_create(scr);
        lv_point_precise_t p[2] = {{0, 0}, {0, 0}};
        memcpy(divPts[i], p, sizeof(p));
        lv_line_set_points(colDividerLine[i], divPts[i], 2);
        lv_obj_add_flag(colDividerLine[i], LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(colDividerLine[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // ---------------- Middle row: Left (Speed), Center (Dimmed Map), Right (Speed Limit) ----------------
    // Left: Tốc độ hiện tại (Glass Card)
    lv_obj_t *speedPane = makePane(scr, 12, 44, 138, 180);
    lv_obj_set_style_bg_opa(speedPane, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(speedPane, 0, 0);

    speedLabel = lv_label_create(speedPane);
    lv_obj_set_style_text_font(speedLabel, &lv_font_montserrat_speed, 0);
    lv_obj_align(speedLabel, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(speedLabel, "0");

    kmhCaption = lv_label_create(speedPane);
    lv_label_set_text(kmhCaption, "");
    lv_obj_add_flag(kmhCaption, LV_OBJ_FLAG_HIDDEN);

    // Right: Biển báo tốc độ cho phép (Glass Card)
    lv_obj_t *signPane = makePane(scr, scrW - 138 - 12, 44, 138, 180);
    lv_obj_set_style_bg_opa(signPane, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(signPane, 0, 0);

    static const int kSignDiam = 104; // 88 -> 104 (2026-09-27, "tang kich thuoc bien bao gioi han toc do")
    speedLimitSign = lv_obj_create(signPane);
    lv_obj_set_size(speedLimitSign, kSignDiam, kSignDiam);
    lv_obj_align(speedLimitSign, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(speedLimitSign, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(speedLimitSign, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(speedLimitSign, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(speedLimitSign, lv_color_hex(0xE60000), 0);
    lv_obj_set_style_border_width(speedLimitSign, 11, 0);
    lv_obj_set_style_shadow_color(speedLimitSign, lv_color_hex(0x000000), 0);
    lv_obj_set_style_shadow_width(speedLimitSign, 12, 0);
    lv_obj_set_style_shadow_opa(speedLimitSign, LV_OPA_50, 0);
    lv_obj_set_style_pad_all(speedLimitSign, 0, 0);
    lv_obj_clear_flag(speedLimitSign, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(speedLimitSign, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(speedLimitSign, LV_OBJ_FLAG_HIDDEN);

    speedLimitValueLabel = lv_label_create(signPane);
    lv_obj_set_style_text_font(speedLimitValueLabel, &lv_font_montserrat_36, 0);
    lv_obj_set_width(speedLimitValueLabel, kSignDiam - 20);
    lv_label_set_long_mode(speedLimitValueLabel, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(speedLimitValueLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(speedLimitValueLabel, speedLimitSign, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(speedLimitValueLabel, "--");

    // Hidden floating labels
    // (legacy floating warning banners removed — see comment near the top of this file)

    // ---------------- Bottom row: Cảnh báo phụ (Traffic Card) ----------------
    lv_obj_t *bottomBar = makePane(scr, 0, scrH - kBottomBarH, scrW, kBottomBarH);
    gBottomBar = bottomBar;
    lv_obj_set_style_border_color(bottomBar, lv_color_hex(0x182232), 0); // same line as the top bar
    lv_obj_set_style_border_width(bottomBar, 1, 0);
    lv_obj_set_style_border_side(bottomBar, LV_BORDER_SIDE_TOP, 0);

    // WiFi status icon, centre of the bottom bar (heading letter left, board
    // temperature right): hidden = WiFi off, grey = hotspot on, green
    // "WiFi ✓" = a phone is connected (refreshDashboard, state-change only).
    wifiTopIcon = lv_label_create(bottomBar);
    lv_obj_set_style_text_font(wifiTopIcon, &lv_font_montserrat_20, 0);
    lv_label_set_text(wifiTopIcon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(wifiTopIcon, lv_color_hex(0x7C8A9A), 0);
    lv_obj_set_width(wifiTopIcon, 60);
    lv_obj_set_style_text_align(wifiTopIcon, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(wifiTopIcon, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(wifiTopIcon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(wifiTopIcon, LV_OBJ_FLAG_HIDDEN);

    midCol = makePane(scr, (scrW - 270) / 2, scrH - kBottomBarH - 6 - 60, 270, 60);
    buildTrafficCard(midCol, 270, 60);

    // Bottom info bar (hidden)
    bottomInfoLabel = lv_label_create(scr);
    lv_obj_align(bottomInfoLabel, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(bottomInfoLabel, LV_OBJ_FLAG_HIDDEN);
}

// ---------------------------------------------------------------------
// Portrait content (rotation 0/2, 320x480) — a different ARRANGEMENT of the
// exact same widgets/information as buildDashboardLandscape() above, not a
// parametric reflow of it (see this file's own layout-constants comment for
// why). Stacked top-to-bottom: status bar, then speed+limit-sign side by
// side, then the sign/camera alert card (given the most vertical room —
// used to be a receding radar road/target view here before radar was
// removed 2026-09-21; the alert card fills that same freed space now).
// ---------------------------------------------------------------------
static void buildDashboardPortrait(lv_obj_t *scr) {
#ifdef VIETHUD_P4
    const int scrW = 320, scrH = 533; // P4 design screen (x1.5 = 480x800)
#else
    const int scrW = 320, scrH = 480;
#endif
    const int topH = 36;

    // ---------------- Top status bar (Full width 320, Glassmorphism) ----------------
    lv_obj_t *topBar = makePane(scr, 0, 0, scrW, topH);
    gTopBar = topBar;
    lv_obj_set_style_bg_opa(topBar, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(topBar, lv_color_hex(0x182232), 0);
    lv_obj_set_style_border_width(topBar, 1, 0);
    lv_obj_set_style_border_side(topBar, LV_BORDER_SIDE_BOTTOM, 0);

    // Left: GNSS status with neon dot
    gnssIcon = lv_label_create(topBar);
    lv_label_set_text(gnssIcon, LV_SYMBOL_GPS);
    lv_obj_align(gnssIcon, LV_ALIGN_LEFT_MID, 10, 0);

    gnssCaption = lv_label_create(topBar);
    lv_obj_set_style_text_font(gnssCaption, &lv_font_vn_14, 0);
    lv_label_set_text(gnssCaption, "--");
    lv_obj_align_to(gnssCaption, gnssIcon, LV_ALIGN_OUT_RIGHT_MID, 6, 0);

    // Right: settings, wifi, clock, sun
    gearIcon = lv_label_create(topBar);
    lv_label_set_text(gearIcon, LV_SYMBOL_SETTINGS);
    lv_obj_align(gearIcon, LV_ALIGN_RIGHT_MID, -10, 0);

    wifiTopIcon = lv_label_create(topBar);
    lv_label_set_text(wifiTopIcon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(wifiTopIcon, lv_color_hex(0x7C8A9A), 0);
    // Fixed width, right-aligned: the text grows to "WiFi ✓" when a phone joins,
    // and the clock is aligned to this box once at build time.
    lv_obj_set_width(wifiTopIcon, 44);
    lv_obj_set_style_text_align(wifiTopIcon, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align_to(wifiTopIcon, gearIcon, LV_ALIGN_OUT_LEFT_MID, -8, 0);
    lv_obj_clear_flag(wifiTopIcon, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(wifiTopIcon, LV_OBJ_FLAG_HIDDEN);

    clockLabel = lv_label_create(topBar);
    lv_obj_set_style_text_font(clockLabel, &lv_font_vn_14, 0);
    lv_obj_align_to(clockLabel, wifiTopIcon, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    lv_label_set_text(clockLabel, "--:--");
    audioTopIcon = lv_label_create(topBar);
    lv_obj_set_style_text_font(audioTopIcon, &lv_font_montserrat_14, 0);
    lv_obj_align_to(audioTopIcon, clockLabel, LV_ALIGN_OUT_LEFT_MID, -8, 0);
    updateAudioTopIcon();

    sunIcon = makeIcon(topBar, &sun_icon);
    lv_obj_align_to(sunIcon, clockLabel, LV_ALIGN_OUT_LEFT_MID, -6, 0);
    // Center: Street Name Badge (Portrait)
    streetNameBadge = lv_obj_create(topBar);
    lv_obj_set_size(streetNameBadge, 160, 24);
    lv_obj_align(streetNameBadge, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(streetNameBadge, lv_color_hex(0x0C1522), 0);
    lv_obj_set_style_bg_opa(streetNameBadge, LV_OPA_80, 0);
    lv_obj_set_style_border_color(streetNameBadge, lv_color_hex(0x1F314A), 0);
    lv_obj_set_style_border_width(streetNameBadge, 1, 0);
    lv_obj_set_style_radius(streetNameBadge, 12, 0);
    lv_obj_set_style_pad_all(streetNameBadge, 0, 0);
    lv_obj_clear_flag(streetNameBadge, LV_OBJ_FLAG_SCROLLABLE);

    streetNameIcon = lv_label_create(streetNameBadge);
    lv_label_set_text(streetNameIcon, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(streetNameIcon, lv_color_hex(0x00E5FF), 0);
    lv_obj_align(streetNameIcon, LV_ALIGN_LEFT_MID, 6, 0);

    streetNameLabel = lv_label_create(streetNameBadge);
    lv_obj_set_style_text_font(streetNameLabel, &lv_font_vn_14, 0);
    lv_obj_set_style_text_color(streetNameLabel, lv_color_hex(0xF0F4F8), 0);
    lv_label_set_long_mode(streetNameLabel, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(streetNameLabel, 125);
    lv_obj_align(streetNameLabel, LV_ALIGN_LEFT_MID, 22, 0);
    lv_label_set_text(streetNameLabel, "");
    lv_obj_add_flag(streetNameBadge, LV_OBJ_FLAG_HIDDEN);

    // Line placeholders for theme compatibility
    static lv_point_precise_t divPts[2][2];
    for (int slot = 0; slot < 2; slot++) {
        lv_obj_t *line = lv_line_create(scr);
        lv_point_precise_t local[2] = {{0, 0}, {0, 0}};
        memcpy(divPts[slot], local, sizeof(local));
        lv_line_set_points(line, divPts[slot], 2);
        lv_obj_add_flag(line, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
        colDividerLine[slot] = line;
    }

    // ---------------- Hàng thứ hai: Tốc độ hiện tại & Biển báo tốc độ đặt cạnh nhau, thẳng hàng ----------------
    lv_obj_t *speedPane = makePane(scr, 12, 44, 138, 120);
    lv_obj_set_style_bg_opa(speedPane, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(speedPane, 0, 0);

    speedLabel = lv_label_create(speedPane);
    lv_obj_set_style_text_font(speedLabel, &lv_font_montserrat_speed, 0);
    lv_obj_align(speedLabel, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(speedLabel, "0");

    kmhCaption = lv_label_create(speedPane);
    lv_label_set_text(kmhCaption, "");
    lv_obj_add_flag(kmhCaption, LV_OBJ_FLAG_HIDDEN);

    // Biển báo tốc độ cho phép (đặt cùng hàng Y=44, chiều cao 120)
    lv_obj_t *signPane = makePane(scr, scrW - 138 - 12, 44, 138, 120);

    static const int kSignDiam = 104; // 88 -> 104 (2026-09-27, "tang kich thuoc bien bao gioi han toc do")
    speedLimitSign = lv_obj_create(signPane);
    lv_obj_set_size(speedLimitSign, kSignDiam, kSignDiam);
    lv_obj_align(speedLimitSign, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(speedLimitSign, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(speedLimitSign, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(speedLimitSign, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(speedLimitSign, lv_color_hex(0xE60000), 0);
    lv_obj_set_style_border_width(speedLimitSign, 11, 0);
    lv_obj_set_style_shadow_color(speedLimitSign, lv_color_hex(0x000000), 0);
    lv_obj_set_style_shadow_width(speedLimitSign, 12, 0);
    lv_obj_set_style_shadow_opa(speedLimitSign, LV_OPA_50, 0);
    lv_obj_set_style_pad_all(speedLimitSign, 0, 0);
    lv_obj_clear_flag(speedLimitSign, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(speedLimitSign, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(speedLimitSign, LV_OBJ_FLAG_HIDDEN);

    speedLimitValueLabel = lv_label_create(signPane);
    lv_obj_set_style_text_font(speedLimitValueLabel, &lv_font_montserrat_36, 0);
    lv_obj_set_width(speedLimitValueLabel, kSignDiam - 20);
    lv_label_set_long_mode(speedLimitValueLabel, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_align(speedLimitValueLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(speedLimitValueLabel, speedLimitSign, LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(speedLimitValueLabel, "--");

    // Hidden floating labels for compatibility
    // (legacy floating warning banners removed — see comment near the top of this file)

    // ---------------- Hàng đáy: Cảnh báo phụ (camera/đổi tốc độ/khoảng cách) ----------------
    midCol = makePane(scr, 25, 252, scrW - 50, 60);
    buildTrafficCard(midCol, scrW - 50, 60);

    // Bottom info bar (hidden)
    bottomInfoLabel = lv_label_create(scr);
    lv_obj_align(bottomInfoLabel, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_add_flag(bottomInfoLabel, LV_OBJ_FLAG_HIDDEN);
}

void buildDashboard() {
    lastTouchMs = millis(); // burn-in idle timer starts counting from boot

    dashboardScreen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(dashboardScreen, lv_color_hex(0x000000), 0);
    lv_obj_set_style_pad_all(dashboardScreen, 0, 0);
    lv_obj_clear_flag(dashboardScreen, LV_OBJ_FLAG_SCROLLABLE);

    // Root container: holds the whole dashboard so the burn-in pixel shift
    // can nudge everything at once, and so a long press anywhere on it opens
    // Settings (spec section 17 suggests long-press for the settings entry).
    int scrW = gfx->width(), scrH = gfx->height(); // dynamic — LS_SCR_W/LS_SCR_H below are landscape-only constants, wrong for portrait
    dashRoot = lv_obj_create(dashboardScreen);
    lv_obj_set_pos(dashRoot, 0, 0);
    lv_obj_set_size(dashRoot, scrW, scrH);
    lv_obj_set_style_bg_color(dashRoot, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(dashRoot, 0, 0);
    lv_obj_set_style_radius(dashRoot, 0, 0);
    lv_obj_set_style_pad_all(dashRoot, 0, 0);
    lv_obj_clear_flag(dashRoot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(dashRoot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(dashRoot, onDashPressed, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(dashRoot, onDashLongPressed, LV_EVENT_LONG_PRESSED, NULL);
    lv_obj_add_event_cb(dashRoot, onDashLongPressedRepeat, LV_EVENT_LONG_PRESSED_REPEAT, NULL);
    lv_obj_add_event_cb(dashRoot, onDashReleasedOrLost, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(dashRoot, onDashReleasedOrLost, LV_EVENT_PRESS_LOST, NULL);

    buildMapCanvas(dashRoot, scrW, scrH);
    lv_obj_t *scr = dashRoot;

    if (scrH > scrW) {
        buildDashboardPortrait(scr);
    } else {
        buildDashboardLandscape(scr);
    }

    // Full-screen speeding overlay (user-requested 2026-09-16) — translucent,
    // not opaque, so speed/limit/etc stay readable through it. Toggled via
    // the HIDDEN flag on its own blink cadence in refreshDashboard() —
    // gated so the color/visibility only change when they actually need to,
    // never every 150ms tick: a full-screen invalidation is the single most
    // expensive thing this UI can ask the Canvas driver to do (see
    // DisplayDriver.h's full-frame-flush note). Used to share this pattern
    // with a TTC-based warningFlashOverlay (red/amber, imminent-collision
    // risk) — removed 2026-09-21 alongside radar itself, so speeding is now
    // the only full-screen flash this Dashboard has.
    speedingFlashOverlay = lv_obj_create(scr);
    lv_obj_set_pos(speedingFlashOverlay, 0, 0);
    lv_obj_set_size(speedingFlashOverlay, scrW, scrH);
    lv_obj_set_style_border_width(speedingFlashOverlay, 0, 0);
    lv_obj_set_style_radius(speedingFlashOverlay, 0, 0);
    lv_obj_set_style_bg_color(speedingFlashOverlay, lv_color_hex(0xB33DC6), 0);
    lv_obj_set_style_bg_opa(speedingFlashOverlay, LV_OPA_30, 0);
    lv_obj_clear_flag(speedingFlashOverlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(speedingFlashOverlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(speedingFlashOverlay, LV_OBJ_FLAG_HIDDEN);

    // Positioned at the FULL screen's top-center (scr, not the alert card —
    // one shared object, built once here, works unchanged in either
    // orientation). Used to stack below a radar "sudden closing speed"
    // banner (y=28 vs. that banner's y=2) — that banner is gone with radar
    // (2026-09-21), so this now sits at the top spot itself. Text set live
    // in refreshDashboard() (the actual upcoming limit number isn't known
    // until then).
    // Top floating banners relocated to dedicated HUD cards to prevent blocking Top Status Bar
    // (legacy floating warning banners removed — see comment near the top of this file)

    // Hold-to-open-Settings progress ring — created last so it draws on top
    // of everything else, including the flash overlay above. Hidden until
    // a press starts; see onDashPressed().
    holdRing = lv_arc_create(scr);
    lv_obj_set_size(holdRing, kHoldRingDiam, kHoldRingDiam);
    lv_arc_set_bg_angles(holdRing, 0, 360);
    lv_arc_set_rotation(holdRing, 270);
    lv_arc_set_range(holdRing, 0, 100);
    lv_arc_set_value(holdRing, 0);
    lv_obj_remove_style(holdRing, NULL, LV_PART_KNOB);
    lv_obj_set_style_arc_width(holdRing, 5, LV_PART_MAIN);
    lv_obj_set_style_arc_width(holdRing, 5, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(holdRing, lv_color_hex(0x2A3441), LV_PART_MAIN);
    lv_obj_set_style_arc_color(holdRing, lv_color_hex(0x4AA3FF), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(holdRing, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_clear_flag(holdRing, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(holdRing, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(holdRing, LV_OBJ_FLAG_HIDDEN);

    // WiFi on/off confirmation toast (user-requested 2026-09-14 alongside
    // the WiFi hold gesture, now the 4s tier) — the gesture itself has no
    // other on-screen feedback (unlike the 1s Settings-open point, which
    // visibly navigates away, or the 2s Simple-layout point, which visibly
    // swaps the whole layout), so without this a successful WiFi hold would
    // look like nothing happened at all. Self-hiding: shown here, hidden again once
    // wifiToastUntilMs passes, checked each refreshDashboard() tick rather
    // than a dedicated timer/anim — same reasoning burnInTimerCb's own
    // idle-dim check gives for reusing an existing periodic tick instead of
    // adding another one for a rarely-firing check.
    wifiToastLabel = lv_label_create(scr);
    lv_obj_set_style_text_font(wifiToastLabel, &lv_font_vn_20, 0); // VN text (audio toast) + symbols via fallback
    lv_obj_set_style_text_color(wifiToastLabel, lv_color_white(), 0);
    lv_obj_set_style_bg_color(wifiToastLabel, lv_color_hex(0x151C24), 0);
    lv_obj_set_style_bg_opa(wifiToastLabel, LV_OPA_90, 0);
    lv_obj_set_style_pad_all(wifiToastLabel, 12, 0);
    lv_obj_set_style_radius(wifiToastLabel, 8, 0);
    lv_obj_center(wifiToastLabel);
    lv_obj_clear_flag(wifiToastLabel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(wifiToastLabel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(wifiToastLabel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(wifiToastLabel); // above the flash overlay/hold ring too

    applyTheme(cfg.themeMode == 1.0f); // Light only if forced; Auto/Dark start dark until GNSS time says it's day
}


// Day/Night color theme for the chrome — background, captions, primary
// readouts, gear icon, column dividers, bottom info bar (user-requested
// 2026-09-15, driven by the same gnss.daytime sunrise/sunset calc already
// used for the clock icon). The sign/camera alert card (buildTrafficCard())
// stays a consistently dark panel in both themes instead — same reasoning
// most cluster/nav UIs keep their map/camera area dark regardless of day/
// night. Risk/status colors (red/amber/purple, ACCENT_COLOR) also stay
// theme-independent everywhere: changing a safety color's hue between Day
// and Night would undermine the "color only means risk" rule (spec section
// 14.2), not serve it.
static void applyTheme(bool daytime) {
    gLightTheme = daytime;
    const ThemePalette &P = pal();
    lv_color_t bg = lv_color_hex(P.bg);

    lv_obj_set_style_bg_color(dashboardScreen, bg, 0);
    lv_obj_set_style_bg_color(dashRoot, bg, 0);

    lv_obj_set_style_text_color(clockLabel, lv_color_hex(P.primaryText), 0);
    lv_obj_set_style_text_color(gearIcon, lv_color_hex(P.captionText), 0);
    for (int i = 0; i < 2; i++) lv_obj_set_style_line_color(colDividerLine[i], lv_color_hex(P.barLine), 0);
    if (gTopBar) lv_obj_set_style_border_color(gTopBar, lv_color_hex(P.barLine), 0);
    if (gBottomBar) lv_obj_set_style_border_color(gBottomBar, lv_color_hex(P.barLine), 0);

    currentPrimaryTextColor = lv_color_hex(P.primaryText);
    lv_obj_set_style_text_color(speedLabel, currentPrimaryTextColor, 0);
    lv_obj_set_style_text_color(kmhCaption, lv_color_hex(P.captionText), 0);
    lv_obj_set_style_text_color(bottomInfoLabel, lv_color_hex(P.captionText), 0);

    if (streetNameBadge) {
        lv_obj_set_style_bg_color(streetNameBadge, lv_color_hex(P.badgeBg), 0);
        lv_obj_set_style_border_color(streetNameBadge, lv_color_hex(P.badgeBorder), 0);
    }
    if (streetNameLabel) lv_obj_set_style_text_color(streetNameLabel, lv_color_hex(P.badgeText), 0);
    if (streetNameIcon) lv_obj_set_style_text_color(streetNameIcon, lv_color_hex(P.accent), 0);
    if (compassLabel) lv_obj_set_style_text_color(compassLabel, lv_color_hex(P.accent), 0);
    if (egoArrow) lv_obj_set_style_line_color(egoArrow, lv_color_hex(P.accent), 0);
    if (egoHalo) {
        lv_obj_set_style_bg_color(egoHalo, lv_color_hex(P.accent), 0);
        lv_obj_set_style_border_color(egoHalo, lv_color_hex(P.accent), 0);
    }
    if (trafficCard) {
        lv_obj_set_style_bg_color(trafficCard, lv_color_hex(P.cardBg), 0);
        lv_obj_set_style_border_color(trafficCard, lv_color_hex(P.cardBorder), 0);
        lv_obj_set_style_shadow_opa(trafficCard, daytime ? LV_OPA_20 : LV_OPA_60, 0);
    }
    if (alertDistLabel) lv_obj_set_style_text_color(alertDistLabel, lv_color_hex(P.cardText), 0);
    if (alertProgressBar) lv_obj_set_style_bg_color(alertProgressBar, lv_color_hex(P.cardTrack), 0);
    if (wifiToastLabel) {
        lv_obj_set_style_bg_color(wifiToastLabel, lv_color_hex(P.toastBg), 0);
        lv_obj_set_style_text_color(wifiToastLabel, lv_color_hex(P.toastText), 0);
    }
    for (int i = 0; i < 4; i++) {
        if (!gVignette[i]) continue;
        gVignetteGrad[i]->stops[0].color = bg;
        gVignetteGrad[i]->stops[1].color = bg;
        lv_obj_set_style_bg_grad(gVignette[i], gVignetteGrad[i], 0); // re-set -> LVGL refreshes the style
    }
    audioIconInit = false; // re-colour on the next updateAudioTopIcon()
    updateAudioTopIcon();
    gThemeRepaintMap = true;            // repaint the map canvas in the new colours
    lastDrawnMapGeneration = 0;
    Serial.printf("[ui] theme -> %s\n", daytime ? "LIGHT" : "DARK");
}

// GREEN = normal, RED = fault, blinking AMBER = pending/searching — see the
// top-bar build comment. blinkOn flips at ~1Hz (500ms/500ms) off the same
// clock refreshDashboard() already runs on (150ms ticks from simTimerCb),
// no separate lv_anim needed.
#define STATUS_GREEN 0x33CC66
#define STATUS_RED 0xFF3B30
#define STATUS_AMBER 0xE0C020
#define STATUS_AMBER_DIM 0x4A4222
// Distinct hue from every TTC-risk color above (green/amber/red) — see
// speedingFlashOverlay's own declaration comment for why speeding needs its
// own color rather than reusing STATUS_RED.
#define STATUS_SPEEDING 0xB33DC6

// Queues the numbered voice clip for kmh (data/speedmap/sounds/vi/speed/
// <N>.mp3) — ONLY if it exactly matches one of the values actually staged
// there (Task C/E, 2026-09-21). Deliberately does not round/approximate to
// the nearest available number: speaking "80" for an actual 82 km/h limit
// would be misinformation, not a rounding nicety, so an unmatched value
// just gets no spoken number (the tone chime + on-screen text still show
// it exactly).
// Full-sentence prompts (2026-09-28, user request: "Gioi han toc do tiep theo
// la 50 km/h", "Phia truoc co camera giam sat toc do..."), generated by
// tools/gen_voice_prompts.py into /speedmap/sounds/vi/: speed_next/<N>.mp3,
// camera_limit/<N>.mp3, camera_ahead.mp3 — one natural utterance per alert
// instead of two stitched clips. Cards without them keep the old prompts.
static const int kPromptSpeeds[] = {20, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100, 110, 120};
static bool promptSpeedKnown(int kmh) {
    for (size_t i = 0; i < sizeof(kPromptSpeeds) / sizeof(kPromptSpeeds[0]); i++)
        if (kPromptSpeeds[i] == kmh) return true;
    return false;
}
// Whether the card has the new prompt set: checked on first use, and again at
// most once a minute while absent (card swapped / files copied later).
static bool newPromptsAvailable() {
    static int8_t sState = -1;
    static uint32_t sCheckedMs = 0;
    if (sState == 1) return true;
    if (sState == 0 && millis() - sCheckedMs < 60000) return false;
    sCheckedMs = millis();
    sState = sdMgrExists("/speedmap/sounds/vi/camera_ahead.mp3") ? 1 : 0;
    Serial.printf("[audio] full-sentence prompts %s\n", sState ? "found" : "not on card (using old prompts)");
    return sState == 1;
}
// "Giới hạn tốc độ tiếp theo là N km/h". false -> caller plays the old prompt.
static bool queueNextLimitPrompt(float kmh) {
    int k = (int)lroundf(kmh);
    if (!promptSpeedKnown(k) || !newPromptsAvailable()) return false;
    char f[32];
    snprintf(f, sizeof(f), "speed_next/%d.mp3", k);
    audioQueueVoice(f);
    return true;
}
// "Phía trước có camera giám sát tốc độ[, giới hạn N km/h]". false -> old prompt.
static bool queueCameraPrompt(float limitKmh) {
    if (!newPromptsAvailable()) return false;
    int k = (int)lroundf(limitKmh);
    if (limitKmh > 0 && promptSpeedKnown(k)) {
        char f[32];
        snprintf(f, sizeof(f), "camera_limit/%d.mp3", k);
        audioQueueVoice(f);
    } else {
        audioQueueVoice("camera_ahead.mp3");
    }
    return true;
}

static void queueSpeedVoice(float kmh) {
    static const int kKnownSpeeds[] = {20, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100, 120};
    int kmhInt = (int)lroundf(kmh);
    for (size_t i = 0; i < sizeof(kKnownSpeeds) / sizeof(kKnownSpeeds[0]); i++) {
        if (kKnownSpeeds[i] == kmhInt) {
            char speedFile[24];
            snprintf(speedFile, sizeof(speedFile), "speed/%d.mp3", kmhInt);
            audioQueueVoice(speedFile);
            return;
        }
    }
}

void refreshDashboard() {
    // Auto-hide the zoom-level toast — checked here
    // rather than a dedicated timer since refreshDashboard() already runs
    // every 150ms while the Dashboard is visible, same reasoning as every
    // other "reuse the existing tick" gate in this function.
    if (!lv_obj_has_flag(wifiToastLabel, LV_OBJ_FLAG_HIDDEN) && millis() > wifiToastUntilMs) {
        lv_obj_add_flag(wifiToastLabel, LV_OBJ_FLAG_HIDDEN);
    }
    updateAudioTopIcon(); // no-op unless cfg.audioEnabled changed (Settings / portal / hold gesture)

    // Single mutex-protected read per refresh — everything below uses these
    // local copies, never the live shared state (see core/SharedState.h).
    GnssSnapshot gnss = gnssSnapshot();

    // GPS status chimes (2026-09-25): a friendly ascending motif the first time
    // a fix is acquired ("device ready"), and a low descending tone if the fix
    // drops WHILE DRIVING. Edge-detected on gnss.fix (already debounced by
    // GNSS.cpp's gnssFixTimeoutS). The "lost" tone only fires if the vehicle
    // actually moved during this fix session (sMovedThisFix) — so a bench with
    // no antenna, or a parked car, doesn't cry wolf. A 3s cooldown stops any
    // marginal-signal toggling from spamming the speaker.
    {
        static int sLastFix = -1; // -1 = first evaluation, no edge yet
        static uint32_t sLastGpsChimeMs = 0;
        static bool sMovedThisFix = false;
        int fixNow = gnss.fix ? 1 : 0;
        if (fixNow && gnss.egoSpeedKmh > 5.0f) sMovedThisFix = true;
        if (sLastFix != -1 && fixNow != sLastFix && millis() - sLastGpsChimeMs > 3000 && cfg.audioSystem) {
            if (fixNow) {
                audioPlayGpsReady();
                sLastGpsChimeMs = millis();
                sMovedThisFix = false; // new fix session
            } else if (sMovedThisFix) {
                audioPlayGpsLost();
                sLastGpsChimeMs = millis();
            }
        }
        sLastFix = fixNow;
    }

    // Board temperature (ESP32-S3 die sensor) + thermal safety. Shown bottom-
    // right, colour-coded. WARNING (>=kTempWarnC): amber + a one-shot notice.
    // CRITICAL (>=kTempCritC): red BLINKING, a repeating alarm, AND the backlight
    // is dimmed to shed heat (the panel is the board's biggest heat source).
    // temperatureRead() is a quick on-chip ADC read; done every ~2s. Colour is
    // re-applied only when it actually changes, so normal operation never forces
    // a full-screen redraw on the map beneath (only the rare critical blink does).
    if (tempLabel) {
        // Arduino's temperatureRead() always uses the sensor's default range
        // (-10..80 C); above that it SATURATES (read a flat "110.0 C" on a hot
        // windscreen, 2026-09-26). Read with the range that fits: -10..80 C
        // (+-1 C), 20..100 C (+-2 C) or 50..125 C (+-3 C), with hysteresis, and
        // re-read at once in the higher range if the current one saturated.
#ifdef VIETHUD_P4
        // IDF 5 aborts if the legacy temp_sensor driver meets the new one the
        // core links in, so on the P4 use temperature_sensor with the same
        // range-adaptive idea: one handle per range, re-read one range up
        // when the current one saturates, drift back down with hysteresis.
        auto readDieTempC = []() -> float {
            static const int kR[3][2] = {{-10, 80}, {20, 100}, {50, 125}};
            static temperature_sensor_handle_t h[3] = {nullptr, nullptr, nullptr};
            static int range = 0;
            auto readIn = [](int r) {
                float c = NAN;
                if (!h[r]) {
                    temperature_sensor_config_t tc = TEMPERATURE_SENSOR_CONFIG_DEFAULT(kR[r][0], kR[r][1]);
                    if (temperature_sensor_install(&tc, &h[r]) != ESP_OK) return c;
                }
                if (temperature_sensor_enable(h[r]) == ESP_OK) {
                    temperature_sensor_get_celsius(h[r], &c);
                    temperature_sensor_disable(h[r]);
                }
                return c;
            };
            float c = readIn(range);
            if (range == 0 && !(c < 78.0f)) c = readIn(range = 1);
            if (range == 1 && !(c < 98.0f)) c = readIn(range = 2);
            if (range == 0 && c >= 75.0f) range = 1;
            else if (range == 1 && c < 65.0f) range = 0;
            else if (range == 2 && c < 90.0f) range = 1;
            return c;
        };
#else
        auto readDieTempC = []() -> float {
            static temp_sensor_dac_offset_t range = TSENS_DAC_L2;
            auto readIn = [](temp_sensor_dac_offset_t r) {
                float c = NAN;
                temp_sensor_config_t t = TSENS_CONFIG_DEFAULT();
                t.dac_offset = r;
                temp_sensor_set_config(t);
                temp_sensor_start();
                temp_sensor_read_celsius(&c);
                temp_sensor_stop();
                return c;
            };
            float c = readIn(range);
            if (range == TSENS_DAC_L2 && !(c < 80.0f)) c = readIn(range = TSENS_DAC_L1);
            if (range == TSENS_DAC_L1 && !(c < 100.0f)) c = readIn(range = TSENS_DAC_L0);
            if (range == TSENS_DAC_L2 && c >= 75.0f) range = TSENS_DAC_L1;       // next read: wider range
            else if (range == TSENS_DAC_L1 && c < 65.0f) range = TSENS_DAC_L2;   // cooled: back to the precise one
            else if (range == TSENS_DAC_L0 && c < 90.0f) range = TSENS_DAC_L1;
            return c;
        };
#endif
        static uint32_t sLastTempMs = 0;
        static float sTempC = -999.0f;
        static int sLastTier = 0;
        static uint32_t sLastCritAlarmMs = 0;
        static uint32_t sLastTempColor = 0xFFFFFFFFu;
        uint32_t nowT = millis();
        if (sTempC < -900.0f || nowT - sLastTempMs > 2000) {
            sLastTempMs = nowT;
            sTempC = readDieTempC(); // ESP32-S3 internal die temperature, degrees C (range-adaptive)
            g_boardTempC = sTempC;      // publish for the web telemetry (net/WebPortal.cpp)
            char buf[16];
            snprintf(buf, sizeof(buf), "%.0f\xC2\xB0" "C", (double)sTempC); // "NN°C"
            lv_label_set_text(tempLabel, buf);
        }
        int tier = (sTempC >= kTempCritC) ? 2 : (sTempC >= kTempWarnC) ? 1 : 0;
        {
            static uint32_t sLastTempLogMs = 0;
            if (nowT - sLastTempLogMs > 10000) {
                sLastTempLogMs = nowT;
                Serial.printf("[temp] die=%.1fC tier=%d (warn>=%.0f crit>=%.0f)\n", (double)sTempC, tier,
                              (double)kTempWarnC, (double)kTempCritC);
            }
        }
        lv_color_t col;
        if (tier == 2) col = ((nowT / 400) % 2 == 0) ? lv_color_hex(0xFF3B30) : lv_color_hex(0x601515); // blink
        else if (tier == 1) col = lv_color_hex(0xE0A000);                                              // amber
        else col = lv_color_hex(pal().neutral);                                                        // neutral
        uint32_t cu = lv_color_to_u32(col);
        if (cu != sLastTempColor) { sLastTempColor = cu; lv_obj_set_style_text_color(tempLabel, col, 0); }
        if (tier > sLastTier && cfg.audioSystem) {           // crossing UP into a hotter tier
            if (tier == 1) audioPlaySignNotice();            // gentle "getting hot" notice
            else if (tier == 2) audioPlayOverspeedAlert();   // urgent alarm entering critical
        }
        sLastTier = tier;
        if (tier == 2) {
            if (nowT - sLastCritAlarmMs > 15000) { sLastCritAlarmMs = nowT; if (cfg.audioSystem) audioPlayOverspeedAlert(); }
            if (!gThermalDimmed) { gThermalDimmed = true; applyConfig(); } // ~16% to cut heat
        } else if (gThermalDimmed) {
            gThermalDimmed = false;
            applyConfig(); // cooled below critical — restore the configured brightness
        }
    }

    // Compass: place each cardinal LETTER toward its true geographic direction.
    // On the heading-up map a bearing B appears at screen-bearing (B - heading)
    // clockwise from straight up, i.e. offset (sin th, -cos th)*R. N=0, E=90,
    // S=180, W=270. Stationary/no heading -> north-up. Only touch on a real
    // change (>1 deg) so it doesn't invalidate the map every tick.
    // Heading readout: show the current travel direction letter. Only recompute
    // while actually moving with a valid heading (>=3 km/h) — at a standstill
    // u-blox heading is noise and would flicker the label between directions;
    // below that we keep the last shown direction. Update (and re-center for the
    // 1- vs 2-char width) only when the direction actually changes.
    if (compassLabel && gnss.fix && gnss.headingValid && gnss.egoSpeedKmh >= 3.0f) {
        static int sLastDir = -1;
        int dir = ((int)lroundf(gnss.headingDeg / 45.0f)) % 8;
        if (dir < 0) dir += 8;
        if (dir != sLastDir) {
            sLastDir = dir;
            lv_label_set_text(compassLabel, kCompass8[dir]);
            // center the bigger glyph: ~6px per character horizontally, ~11px vertically
            lv_obj_set_pos(compassLabel, northCx - (int)strlen(kCompass8[dir]) * 6, northCy - 11);
        }
    }

    // dim-while-stationary gate above only dims in the first place once
    // parked, so waking symmetrically on the same signal (rather than
    // leaving the driver to tap the screen themselves) is the safety-first
    // completion of that same 2026-09-16 request.
    if (screenDimmed && gnss.fix && gnss.egoSpeedKmh > kGnssMotionThresholdKmh) wakeScreen();

    // --- Top bar: color-only status (no OK/FAULT/SEARCH words) ---
    // GNSS has a real third state: module alive and sending valid NMEA but
    // no fix yet (normal while cold-starting/indoors, not an error) versus
    // nothing received at all (an actual wiring/power/baud problem) — see
    // gnss/GNSS.cpp's linkAlive. Blink amber for the pending case so it
    // reads as "in progress" rather than steady-state.
    bool blinkOn = (millis() / 500) % 2 == 0;
    lv_color_t gnssColor;
    char gBuf[24];
    if (gnss.fix && (gnss.fixSeq & 0x80000000u)) {
        // Simulated fix (serial S/W/R drive sims, or the automatic road sim
        // when no GNSS module is fitted): say so plainly — never pass a
        // simulated position off as a real satellite fix.
        gnssColor = lv_color_hex(0xB07CFF);
        snprintf(gBuf, sizeof(gBuf), "SIM");
    } else if (gnss.fix && gnss.fromPhone) {
        // Position from the phone over BLE — say where it comes from.
        gnssColor = lv_color_hex(0x3DA5FF);
        snprintf(gBuf, sizeof(gBuf), "PHONE");
    } else if (gnss.fix) {
        gnssColor = lv_color_hex(STATUS_GREEN);
        snprintf(gBuf, sizeof(gBuf), "%d", gnss.satCount);
    } else if (gnss.linkAlive) {
        gnssColor = blinkOn ? lv_color_hex(STATUS_AMBER) : lv_color_hex(STATUS_AMBER_DIM);
        if (gnss.satCount > 0) {
            snprintf(gBuf, sizeof(gBuf), "%d", gnss.satCount);
        } else {
            snprintf(gBuf, sizeof(gBuf), "--");
        }
    } else {
        gnssColor = lv_color_hex(STATUS_RED);
        snprintf(gBuf, sizeof(gBuf), "--");
    }
    // Only recolor on an actual change (2026-09-24 V2). A style write always
    // invalidates the object even if the value is identical — doing it every
    // 150ms tick was one of several redraws that, over the full-screen rotated
    // map beneath, forced a whole-frame QSPI flush every tick. lv_label_set_text
    // already diffs internally, so it stays unconditional.
    {
        static uint32_t sLastGnssColor = 0xFFFFFFFFu;
        uint32_t gc = lv_color_to_u32(gnssColor);
        if (gc != sLastGnssColor) {
            sLastGnssColor = gc;
            lv_obj_set_style_text_color(gnssIcon, gnssColor, 0);
            lv_obj_set_style_text_color(gnssCaption, gnssColor, 0);
        }
    }
    lv_label_set_text(gnssCaption, gBuf); // text itself set once at build — see buildDashboard()

    // WiFi icon, three states: hidden (WiFi off), grey (hotspot on, waiting),
    // green "WiFi ✓" (a phone is connected to the hotspot). Touched only on a
    // state CHANGE — a style/text write always invalidates, and this runs 6-7x/s.
    // State 1 BLINKS (amber, ~1 Hz) so "hotspot on, no phone yet" is noticeable;
    // only the small icon area is invalidated, and only at the blink edges.
    static int lastWifiState = 0; // 0 off, 1 on/waiting (blinking), 2 phone connected
    int wifiState = !webPortalIsEnabled() ? 0 : (webPortalClientCount() > 0 ? 2 : 1);
    static bool sWifiBlinkOn = true;
    if (wifiState != lastWifiState) {
        lastWifiState = wifiState;
        sWifiBlinkOn = true;
        if (wifiState == 0) {
            lv_obj_add_flag(wifiTopIcon, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_label_set_text(wifiTopIcon, wifiState == 2 ? LV_SYMBOL_WIFI " " LV_SYMBOL_OK : LV_SYMBOL_WIFI);
            lv_obj_set_style_text_color(wifiTopIcon, lv_color_hex(wifiState == 2 ? 0x3CC46E : 0xE5B53A), 0);
            lv_obj_set_style_opa(wifiTopIcon, LV_OPA_COVER, 0);
            lv_obj_clear_flag(wifiTopIcon, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (wifiState == 1) {
        bool on = (millis() / 500) % 2 == 0;
        if (on != sWifiBlinkOn) {
            sWifiBlinkOn = on;
            lv_obj_set_style_opa(wifiTopIcon, on ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        }
    }

    // Real local time from the GNSS fix (spec section 15.2 wants sunrise/
    // sunset off GNSS date/time/position eventually — this is the time
    // half). GPS gives UTC only; local offset is estimated from longitude
    // (round(lon/15)), which is exact for this deployment (Vietnam, UTC+7)
    // and a reasonable approximation generally without a timezone DB.
    int ntpH, ntpM;
    if (gnss.timeValid) {
        int offsetHours = (int)lroundf(gnss.lonDeg / 15.0f);
        int localHour = ((gnss.utcHour + offsetHours) % 24 + 24) % 24;
        char buf[8];
        snprintf(buf, sizeof(buf), "%02d:%02d", localHour, gnss.utcMinute);
        lv_label_set_text(clockLabel, buf);
    } else if (webPortalLocalTime(&ntpH, &ntpM)) {
        // No GPS time yet, but WiFi station + NTP gave us the time (feature F).
        char buf[8];
        snprintf(buf, sizeof(buf), "%02d:%02d", ntpH, ntpM);
        lv_label_set_text(clockLabel, buf);
    } else {
        lv_label_set_text(clockLabel, "--:--");
    }
    // Sun during the day, moon at night — gnss.daytime comes from GNSS.cpp's
    // sunrise/sunset calc off real date/time/lat/lon (spec section 15.2).
    // Defaults to sun (daytime=true) while timeValid is false, so this is
    // safe to call unconditionally before any fix.
    {
        static int sLastDay = -1; // only swap the icon (an invalidating op) when day/night flips
        if ((int)gnss.daytime != sLastDay) {
            sLastDay = (int)gnss.daytime;
            lv_image_set_src(sunIcon, gnss.daytime ? &sun_icon : &moon_icon);
        }
    }

    // Theme mode (user-requested 2026-09-15, Settings > Display): Auto
    // follows the real sunrise/sunset calc below unchanged; Light/Dark
    // override it either way. Re-themes only on an actual CHANGE in the
    // resulting effective value (either gnss.daytime changing under Auto,
    // or cfg.themeMode itself changing) — touching lv_obj_set_style_*
    // unconditionally every 150ms would mean re-invalidating the whole
    // screen background + every themed label on every tick for nothing.
    // Auto brightness: re-apply the backlight when day/night flips or the mode
    // changes (applyConfig() itself applies the 50 % night cap).
    {
        static int sLastBriNight = -1;
        gIsNight = !gnss.daytime;
        int briNight = (cfg.brightnessMode < 0.5f && gIsNight) ? 1 : 0;
        if (briNight != sLastBriNight) {
            sLastBriNight = briNight;
            applyConfig();
            Serial.printf("[display] brightness %s\n", briNight ? "night cap 50%" : "normal");
        }
    }

    // 0 = Auto: Light by day, Dark by night (GNSS sunrise/sunset). Without a
    // GNSS clock yet the day/night isn't known -> stay Dark (no glare at night).
    bool effectiveDaytime = cfg.themeMode == 1.0f    ? true
                             : cfg.themeMode == 2.0f ? false
                                                      : (gnss.timeValid && gnss.daytime);
    static bool lastDaytime = true;
    static float lastThemeMode = -1; // forces the very first tick to apply, whatever cfg.themeMode loaded as
    if (effectiveDaytime != lastDaytime || cfg.themeMode != lastThemeMode) {
        lastDaytime = effectiveDaytime;
        lastThemeMode = cfg.themeMode;
        applyTheme(effectiveDaytime);
    }

    // --- Left column ---
    // Speed limit read first: whether the driver is currently speeding
    // decides speedLabel's own color just below, so the compliance check
    // needs to happen before speedLabel is touched. map/SpeedLimitManager.cpp
    // only ever sets RoadInfoSnapshot.valid once it has BOTH a loaded map
    // and a current GNSS fix matched against it — checking .valid alone is
    // suffient, no separate gnss.fix check needed for the sign itself.
    RoadInfoSnapshot road = roadInfoSnapshot();
    // User requirement: độ lệch cảnh báo quá tốc độ là 1km, cứ lớn hơn là cảnh báo
    float overspeedThreshold = road.speedLimitKmh + cfg.overspeedOffsetKmh;
    bool speeding = road.valid && gnss.fix && road.speedLimitKmh > 0 &&
                    (gnss.egoSpeedKmh > overspeedThreshold);

    // Audio overspeed chime & voice alert
    static bool lastSpeeding = false;
    static uint32_t lastSpeedingAudioMs = 0;
    if (speeding) {
        uint32_t now = millis();
        if (cfg.audioOverspeed && (!lastSpeeding || (now - lastSpeedingAudioMs > 8000))) {
            lastSpeedingAudioMs = now;
            audioPlayOverspeedAlert();
            audioQueueVoice("slowdown/voice.mp3");
        }
    }
    if (speeding != lastSpeeding)
        Serial.printf("[speed] %s v=%.0f lim=%d thr=%.0f\n", speeding ? "OVER" : "ok", (double)gnss.egoSpeedKmh,
                      (int)road.speedLimitKmh, (double)overspeedThreshold);
    lastSpeeding = speeding;

    if (gnss.fix) {
        // NOTE: LVGL's builtin vsnprintf has %f support compiled out when
        // LV_USE_FLOAT=0 (our lv_conf.h) — passing %f to lv_label_set_text_fmt
        // silently corrupts the varargs and crashes (LoadProhibited).
        // Confirmed on real hardware 2026-09-14. Format floats with the real
        // libc snprintf into a buffer instead, then set the plain string.
        char buf[16];
        snprintf(buf, sizeof(buf), "%.0f", (double)gnss.egoSpeedKmh);
        lv_label_set_text(speedLabel, buf);
    } else {
        lv_label_set_text(speedLabel, "--");
    }
    // Overspeed is the "risk engine" side of map matching, not just a
    // passive sign readout: the EGO speed itself turns red once over the
    // matched segment's limit (a real sign has fixed colors — see its own
    // build comment — so the warning has to live somewhere else). This is
    // the direct replacement for what used to be the sign's own compliance
    // color before the sign became an authentic, fixed-color regulatory
    // sign.
    {
        static uint32_t sLastSpeedColor = 0xFFFFFFFFu; // only recolor the big speed number on a change
        lv_color_t sColor = speeding ? lv_color_hex(STATUS_RED) : currentPrimaryTextColor;
        uint32_t su = lv_color_to_u32(sColor);
        if (su != sLastSpeedColor) {
            sLastSpeedColor = su;
            lv_obj_set_style_text_color(speedLabel, sColor, 0);
        }
    }

    // Full-screen speeding overlay (user-requested 2026-09-16, "neu vuot
    // qua toc do toi da thi cung canh bao bang layer mau toan man hinh",
    // then "qua toc do cung nhap nhay de tang su chu y"). Blinks at 400ms —
    // its own distinct rate/color (0xB33DC6) so it's never mistaken for a
    // routine sign/camera banner. Same "only touch the flag on an actual
    // blink-phase flip" gating as every other flash/banner in this
    // function, not unconditionally every 150ms tick.
    {
        static bool lastSpeedingVisible = false;
        bool speedingFlashOn = speeding && (millis() / 400) % 2 == 0;
        if (speedingFlashOn != lastSpeedingVisible) {
            lastSpeedingVisible = speedingFlashOn;
            if (speedingFlashOn) lv_obj_clear_flag(speedingFlashOverlay, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(speedingFlashOverlay, LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Speed-limit sign: hidden entirely (not just showing "--" inside a
    // ring) when there's no valid match — a real sign has no "unknown"
    // state to render. Re-centers the value label on every update (not
    // just once at build time) since "60" and "100" aren't the same width
    // and a stale center would look off — align_to recomputes from the
    // sign's actual geometry, same "no guessed offsets" rule this file
    // uses everywhere else.
    // Speed-limit sign: always visible as an authentic P.127 regulatory sign.
    lv_obj_clear_flag(speedLimitSign, LV_OBJ_FLAG_HIDDEN);
    if (road.valid && road.speedLimitKmh > 0) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%.0f", (double)road.speedLimitKmh);
        lv_label_set_text(speedLimitValueLabel, buf);
    } else {
        lv_label_set_text(speedLimitValueLabel, "--");
    }
    {
        // 48 px digits for 2-digit limits; 3 digits (100/120) keep 36 px to fit the ring.
        static int sLastLimitFont = -1;
        int want = strlen(lv_label_get_text(speedLimitValueLabel)) >= 3 ? 36 : 48;
        if (want != sLastLimitFont) {
            sLastLimitFont = want;
            lv_obj_set_style_text_font(speedLimitValueLabel, want == 48 ? &lv_font_montserrat_48 : &lv_font_montserrat_36, 0);
        }
    }
    {
        static bool sLimitColorSet = false; // the sign value is always black — set it once, not every tick
        if (!sLimitColorSet) {
            sLimitColorSet = true;
            lv_obj_set_style_text_color(speedLimitValueLabel, lv_color_black(), 0);
        }
    }
    lv_obj_align_to(speedLimitValueLabel, speedLimitSign, LV_ALIGN_CENTER, 0, 0);

    // Upcoming speed-limit-change banner (see aheadLimitLabel's own
    // declaration comment) — gated on an actual state CHANGE (visibility
    // OR the number itself), same discipline as every other banner/flash
    // in this function, not just set unconditionally every 150ms tick.
    static bool lastAheadVisible = false;
    static float lastAheadLimit = -1;
    if (road.aheadLimitValid != lastAheadVisible || road.aheadSpeedLimitKmh != lastAheadLimit) {
        bool newlyVisible = (road.aheadLimitValid && !lastAheadVisible);
        lastAheadVisible = road.aheadLimitValid;
        lastAheadLimit = road.aheadSpeedLimitKmh;
        // Visual is the trafficCard (W_AHEAD_LIMIT); this block only fires the
        // audio cue when a speed-limit change first appears ahead.
        if (road.aheadLimitValid && newlyVisible && cfg.audioLimitAhead) {
            audioPlaySignNotice();
            if (!queueNextLimitPrompt(road.aheadSpeedLimitKmh)) {
                audioQueueVoice("tocdogioihan.mp3");
                queueSpeedVoice(road.aheadSpeedLimitKmh);
            }
        }
    }

    // Speed-camera-ahead banner with audio chime trigger
    static bool lastCameraVisible = false;
    static float lastCameraDist = -1;
    if (road.cameraAheadValid != lastCameraVisible || fabsf(road.cameraAheadDistanceM - lastCameraDist) > 5.0f) {
        bool newlyVisible = (road.cameraAheadValid && !lastCameraVisible);
        lastCameraVisible = road.cameraAheadValid;
        lastCameraDist = road.cameraAheadDistanceM;
        // Visual is the trafficCard (W_CAMERA); this block only fires the audio
        // cue when a camera first appears ahead.
        if (road.cameraAheadValid && newlyVisible && cfg.audioCamera) {
            audioPlayCameraAlert(); // immediate tone chime — cheap, instant, plays while the voice line below queues
            if (!queueCameraPrompt(road.cameraSpeedLimitKmh)) {
                audioQueueVoice("speedcamera.mp3");
                if (road.cameraSpeedLimitKmh >= 0) queueSpeedVoice(road.cameraSpeedLimitKmh);
            }
        }
    }

    // Traffic sign banner (Khu dan cu, Cam vuot, Tram thu phi, Den tin hieu)
    static bool lastSignVisible = false;
    static int lastSignType = 0;
    static float lastSignDist = -1;

    bool signVisible = road.residentAreaAheadValid || road.noOvertakingAheadValid ||
                       road.tollBoothAheadValid || road.trafficLightAheadValid || road.dangerAheadValid;

    int currentSignType = 0;
    float currentSignDist = -1;

    if (road.residentAreaAheadValid) {
        currentSignType = 2;
        currentSignDist = road.residentAreaAheadDistM;
    } else if (road.noOvertakingAheadValid) {
        currentSignType = 3;
        currentSignDist = road.noOvertakingAheadDistM;
    } else if (road.tollBoothAheadValid) {
        currentSignType = 5;
        currentSignDist = road.tollBoothAheadDistM;
    } else if (road.trafficLightAheadValid) {
        currentSignType = 6;
        currentSignDist = road.trafficLightAheadDistM;
    } else if (road.dangerAheadValid) {
        currentSignType = 10;
        currentSignDist = road.dangerAheadDistM;
    }

    if (signVisible != lastSignVisible || currentSignType != lastSignType ||
        fabsf(currentSignDist - lastSignDist) > 5.0f) {
        bool newlyVisible = (signVisible && !lastSignVisible);
        lastSignVisible = signVisible;
        lastSignType = currentSignType;
        lastSignDist = currentSignDist;

        // Visual is the trafficCard (W_RESIDENT/W_NO_OVERTAKE/W_TOLL/W_LIGHT/
        // W_DANGER, icon + distance); this block only fires the audio cue when a
        // sign first appears ahead.
        bool typeAudio = (currentSignType == 2 && cfg.audioResident) || (currentSignType == 3 && cfg.audioNoOvertake) ||
                         (currentSignType == 5 && cfg.audioToll) || (currentSignType == 6 && cfg.audioLight) ||
                         (currentSignType == 10 && cfg.audioDanger);
        if (signVisible && newlyVisible && typeAudio) {
            audioPlaySignNotice(); // immediate tone chime — cheap, instant, plays while the voice line below queues
            // Only resident-area/no-overtaking/toll/traffic-light/danger have
            // a matching voice asset in data/speedmap/sounds/vi/ — no fallback
            // fabricated for anything else; see AudioPlayer.h.
            switch (currentSignType) {
                case 2: audioQueueVoice(road.residentAreaIsStart ? "batdaukhudancu.mp3" : "hetkhudongdancu.mp3"); break;
                case 3: audioQueueVoice(road.noOvertakingIsStart ? "camvuot.mp3" : "hetcamvuot.mp3"); break;
                case 5: audioQueueVoice("tramthuphi.mp3"); break;
                case 6: audioQueueVoice("chuydentinhieugiaothong.mp3"); break;
                case 10: audioQueueVoice("sapdenbienbao.mp3"); break; // generic "sắp đến biển báo" for a hazard zone
                default: break;
            }
        }
    }

    // --- Update Traffic & Camera Alert Card (Minimalist HUD: Icon + Mini Speed Sign + Distance) ---
    if (trafficCard) {
        // Pick the GENUINELY NEXT alert: the one with the smallest remaining
        // distance, not a fixed type priority. Every distance here is now an
        // along-route distance (map/RoutePredictor.h projects each camera/sign
        // onto the forward route and measures how far the car actually drives
        // to reach it), so "smallest" is truly the next thing on the road — a
        // camera 300m ahead no longer hides a resident-area sign 40m ahead just
        // because camera used to win by type. Ties keep the old order via
        // strict-less-than comparisons below (camera, then limit-change, then
        // resident/no-overtake/toll/light).
        enum { W_NONE, W_CAMERA, W_AHEAD_LIMIT, W_RESIDENT, W_NO_OVERTAKE, W_TOLL, W_LIGHT, W_DANGER };
        int winner = W_NONE;
        float winnerDist = 1e9f;
        if (road.cameraAheadValid && road.cameraAheadDistanceM < winnerDist) {
            winner = W_CAMERA; winnerDist = road.cameraAheadDistanceM;
        }
        if (road.aheadLimitValid && road.aheadSpeedLimitKmh > 0 && road.aheadDistanceM < winnerDist) {
            winner = W_AHEAD_LIMIT; winnerDist = road.aheadDistanceM;
        }
        if (road.residentAreaAheadValid && road.residentAreaAheadDistM < winnerDist) {
            winner = W_RESIDENT; winnerDist = road.residentAreaAheadDistM;
        }
        if (road.noOvertakingAheadValid && road.noOvertakingAheadDistM < winnerDist) {
            winner = W_NO_OVERTAKE; winnerDist = road.noOvertakingAheadDistM;
        }
        if (road.tollBoothAheadValid && road.tollBoothAheadDistM < winnerDist) {
            winner = W_TOLL; winnerDist = road.tollBoothAheadDistM;
        }
        if (road.trafficLightAheadValid && road.trafficLightAheadDistM < winnerDist) {
            winner = W_LIGHT; winnerDist = road.trafficLightAheadDistM;
        }
        if (road.dangerAheadValid && road.dangerAheadDistM < winnerDist) {
            winner = W_DANGER; winnerDist = road.dangerAheadDistM;
        }

        if (winner == W_CAMERA) {
            // Speed Camera / Traffic Camera
            lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(alertIconImg, LV_OBJ_FLAG_HIDDEN);
            lv_image_set_src(alertIconImg, &camera_icon);

            if (road.cameraSpeedLimitKmh > 0) {
                // Camera WITH Speed Limit: Show Camera Icon + Mini Speed Sign
                lv_obj_align(alertIconImg, LV_ALIGN_LEFT_MID, 16, 0);
                lv_obj_clear_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);
                lv_obj_align(alertMiniSpeedSign, LV_ALIGN_LEFT_MID, 58, 0);

                char sBuf[8];
                snprintf(sBuf, sizeof(sBuf), "%.0f", (double)road.cameraSpeedLimitKmh);
                lv_label_set_text(alertMiniSpeedVal, sBuf);
                if (road.cameraSpeedLimitKmh >= 100) {
                    lv_obj_set_style_text_font(alertMiniSpeedVal, &lv_font_vn_14, 0);
                } else {
                    lv_obj_set_style_text_font(alertMiniSpeedVal, &lv_font_montserrat_24, 0);
                }
            } else {
                // Camera WITHOUT Speed Limit: Only Camera Icon
                lv_obj_align(alertIconImg, LV_ALIGN_LEFT_MID, 22, 0);
                lv_obj_add_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);
            }

            char dBuf[16];
            snprintf(dBuf, sizeof(dBuf), "%.0f m", (double)road.cameraAheadDistanceM);
            lv_label_set_text(alertDistLabel, dBuf);

            float maxWarnM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
            int prog = (int)((maxWarnM - road.cameraAheadDistanceM) * 100.0f / maxWarnM);
            if (prog < 0) prog = 0;
            if (prog > 100) prog = 100;
            lv_bar_set_range(alertProgressBar, 0, 100);
            lv_bar_set_value(alertProgressBar, prog, LV_ANIM_OFF);
            lv_color_t pColor = (road.cameraAheadDistanceM < 100.0f) ? lv_color_hex(0xFF3B30) : lv_color_hex(0xFF9800);
            lv_obj_set_style_bg_color(alertProgressBar, pColor, LV_PART_INDICATOR);
            lv_obj_set_style_border_color(trafficCard, pColor, 0);

        } else if (winner == W_AHEAD_LIMIT) {
            // Speed Limit Change Ahead -> Mini Speed Sign + Distance
            lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(alertIconImg, LV_OBJ_FLAG_HIDDEN);

            lv_obj_clear_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);
            lv_obj_align(alertMiniSpeedSign, LV_ALIGN_LEFT_MID, 22, 0);

            char sBuf[8];
            snprintf(sBuf, sizeof(sBuf), "%.0f", (double)road.aheadSpeedLimitKmh);
            lv_label_set_text(alertMiniSpeedVal, sBuf);
            if (road.aheadSpeedLimitKmh >= 100) {
                lv_obj_set_style_text_font(alertMiniSpeedVal, &lv_font_vn_14, 0);
            } else {
                lv_obj_set_style_text_font(alertMiniSpeedVal, &lv_font_montserrat_24, 0);
            }

            char dBuf[16];
            snprintf(dBuf, sizeof(dBuf), "%.0f m", (double)road.aheadDistanceM);
            lv_label_set_text(alertDistLabel, dBuf);

            float maxWarnM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
            int prog = (int)((maxWarnM - road.aheadDistanceM) * 100.0f / maxWarnM);
            if (prog < 0) prog = 0;
            if (prog > 100) prog = 100;
            lv_bar_set_range(alertProgressBar, 0, 100);
            lv_bar_set_value(alertProgressBar, prog, LV_ANIM_OFF);
            lv_color_t pColor = (road.aheadDistanceM < 100.0f) ? lv_color_hex(0xFF9800) : lv_color_hex(0x2196F3);
            lv_obj_set_style_bg_color(alertProgressBar, pColor, LV_PART_INDICATOR);
            lv_obj_set_style_border_color(trafficCard, pColor, 0);

        } else if (winner == W_RESIDENT) {
            // Resident Area (Khu dong dan cu) -> Icon + Distance
            lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(alertIconImg, LV_OBJ_FLAG_HIDDEN);
            lv_image_set_src(alertIconImg, &resident_icon);
            lv_obj_align(alertIconImg, LV_ALIGN_LEFT_MID, 22, 0);
            lv_obj_add_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);

            char dBuf[16];
            snprintf(dBuf, sizeof(dBuf), "%.0f m", (double)road.residentAreaAheadDistM);
            lv_label_set_text(alertDistLabel, dBuf);

            float maxWarnM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
            int prog = (int)((maxWarnM - road.residentAreaAheadDistM) * 100.0f / maxWarnM);
            if (prog < 0) prog = 0;
            if (prog > 100) prog = 100;
            lv_bar_set_range(alertProgressBar, 0, 100);
            lv_bar_set_value(alertProgressBar, prog, LV_ANIM_OFF);
            lv_color_t pColor = (road.residentAreaAheadDistM < 100.0f) ? lv_color_hex(0xFF9800) : lv_color_hex(0x1976D2);
            lv_obj_set_style_bg_color(alertProgressBar, pColor, LV_PART_INDICATOR);
            lv_obj_set_style_border_color(trafficCard, pColor, 0);

        } else if (winner == W_NO_OVERTAKE) {
            // No Overtaking -> Icon + Distance
            lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(alertIconImg, LV_OBJ_FLAG_HIDDEN);
            lv_image_set_src(alertIconImg, &no_overtake_icon);
            lv_obj_align(alertIconImg, LV_ALIGN_LEFT_MID, 22, 0);
            lv_obj_add_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);

            char dBuf[16];
            snprintf(dBuf, sizeof(dBuf), "%.0f m", (double)road.noOvertakingAheadDistM);
            lv_label_set_text(alertDistLabel, dBuf);

            float maxWarnM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
            int prog = (int)((maxWarnM - road.noOvertakingAheadDistM) * 100.0f / maxWarnM);
            if (prog < 0) prog = 0;
            if (prog > 100) prog = 100;
            lv_bar_set_range(alertProgressBar, 0, 100);
            lv_bar_set_value(alertProgressBar, prog, LV_ANIM_OFF);
            lv_color_t pColor = (road.noOvertakingAheadDistM < 100.0f) ? lv_color_hex(0xFF3B30) : lv_color_hex(0xE53935);
            lv_obj_set_style_bg_color(alertProgressBar, pColor, LV_PART_INDICATOR);
            lv_obj_set_style_border_color(trafficCard, pColor, 0);

        } else if (winner == W_TOLL) {
            // Toll Booth -> Icon + Distance
            lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(alertIconImg, LV_OBJ_FLAG_HIDDEN);
            lv_image_set_src(alertIconImg, &toll_icon);
            lv_obj_align(alertIconImg, LV_ALIGN_LEFT_MID, 22, 0);
            lv_obj_add_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);

            char dBuf[16];
            snprintf(dBuf, sizeof(dBuf), "%.0f m", (double)road.tollBoothAheadDistM);
            lv_label_set_text(alertDistLabel, dBuf);

            float maxWarnM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
            int prog = (int)((maxWarnM - road.tollBoothAheadDistM) * 100.0f / maxWarnM);
            if (prog < 0) prog = 0;
            if (prog > 100) prog = 100;
            lv_bar_set_range(alertProgressBar, 0, 100);
            lv_bar_set_value(alertProgressBar, prog, LV_ANIM_OFF);
            lv_color_t pColor = (road.tollBoothAheadDistM < 100.0f) ? lv_color_hex(0xFF9800) : lv_color_hex(0x8E24AA);
            lv_obj_set_style_bg_color(alertProgressBar, pColor, LV_PART_INDICATOR);
            lv_obj_set_style_border_color(trafficCard, pColor, 0);

        } else if (winner == W_LIGHT) {
            // Traffic Light / Intersection -> Icon + Distance
            lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(alertIconImg, LV_OBJ_FLAG_HIDDEN);
            lv_image_set_src(alertIconImg, &traffic_light_icon);
            lv_obj_align(alertIconImg, LV_ALIGN_LEFT_MID, 22, 0);
            lv_obj_add_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);

            char dBuf[16];
            snprintf(dBuf, sizeof(dBuf), "%.0f m", (double)road.trafficLightAheadDistM);
            lv_label_set_text(alertDistLabel, dBuf);

            float maxWarnM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
            int prog = (int)((maxWarnM - road.trafficLightAheadDistM) * 100.0f / maxWarnM);
            if (prog < 0) prog = 0;
            if (prog > 100) prog = 100;
            lv_bar_set_range(alertProgressBar, 0, 100);
            lv_bar_set_value(alertProgressBar, prog, LV_ANIM_OFF);
            lv_color_t pColor = (road.trafficLightAheadDistM < 100.0f) ? lv_color_hex(0xFF9800) : lv_color_hex(0x00897B);
            lv_obj_set_style_bg_color(alertProgressBar, pColor, LV_PART_INDICATOR);
            lv_obj_set_style_border_color(trafficCard, pColor, 0);

        } else if (winner == W_DANGER) {
            // Hazard / danger zone (đoạn đường nguy hiểm / hầm / trạm dừng) -> warning icon + distance
            lv_obj_clear_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(alertIconImg, LV_OBJ_FLAG_HIDDEN);
            lv_image_set_src(alertIconImg, &warning_icon);
            lv_obj_align(alertIconImg, LV_ALIGN_LEFT_MID, 24, 0);
            lv_obj_add_flag(alertMiniSpeedSign, LV_OBJ_FLAG_HIDDEN);

            char dBuf[16];
            snprintf(dBuf, sizeof(dBuf), "%.0f m", (double)road.dangerAheadDistM);
            lv_label_set_text(alertDistLabel, dBuf);

            float maxWarnM = computeDynamicWarnDistance(gnss.egoSpeedKmh);
            int prog = (int)((maxWarnM - road.dangerAheadDistM) * 100.0f / maxWarnM);
            if (prog < 0) prog = 0;
            if (prog > 100) prog = 100;
            lv_bar_set_range(alertProgressBar, 0, 100);
            lv_bar_set_value(alertProgressBar, prog, LV_ANIM_OFF);
            lv_color_t pColor = (road.dangerAheadDistM < 120.0f) ? lv_color_hex(0xFF3B30) : lv_color_hex(0xFFB300);
            lv_obj_set_style_bg_color(alertProgressBar, pColor, LV_PART_INDICATOR);
            lv_obj_set_style_border_color(trafficCard, pColor, 0);

        } else {
            // No alert ahead -> completely hide the card!
            lv_obj_add_flag(trafficCard, LV_OBJ_FLAG_HIDDEN);
        }
    }
    // --- Bottom info bar (permanently cleared & hidden) ---
    if (bottomInfoLabel) {
        lv_label_set_text(bottomInfoLabel, "");
        lv_obj_add_flag(bottomInfoLabel, LV_OBJ_FLAG_HIDDEN);
    }

    // --- Street Name Banner ---
    if (streetNameBadge && streetNameLabel) {
        if (road.roadName[0] != '\0') {
            char nameBuf[64];
            abbreviateRoadName(road.roadName, nameBuf, sizeof(nameBuf));
            // Only on a real change: lv_label_set_text() restarts the label's
            // scroll animation, and this runs every 150 ms, so a name longer
            // than the badge never actually scrolled (2026-09-27, "ten duong
            // dai hon o hien thi ... chay tu phai qua trai").
            if (strcmp(lv_label_get_text(streetNameLabel), nameBuf) != 0) lv_label_set_text(streetNameLabel, nameBuf);
            lv_obj_clear_flag(streetNameBadge, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(streetNameBadge, LV_OBJ_FLAG_HIDDEN);
        }
    }

    updateMapCanvas();
}

// Only refresh while the Dashboard is the screen actually on-screen — this
// used to run unconditionally every 150ms even while Settings was open,
// burning CPU on a screen nobody could see. User-reported 2026-09-15 as
// Settings feeling laggy; this was competing with Settings' own touch
// handling for the same Core 1 loop().
void simTimerCb(lv_timer_t *) {
    if (lv_screen_active() == dashboardScreen) refreshDashboard();
}

// Runs once a minute: nudges the whole dashboard by a couple of pixels so no
// static edge burns in, and applies the idle auto-dim. The pixel shift only
// matters while the Dashboard is visible (nothing to un-burn-in on a hidden
// screen); auto-dim stays screen-agnostic — idle is idle whether the driver
// left it on Dashboard or Settings.
void burnInTimerCb(lv_timer_t *) {
    if (lv_screen_active() == dashboardScreen) {
        pixelShiftIdx = (pixelShiftIdx + 1) % 4;
        lv_obj_set_pos(dashRoot, kPixelShiftOffsets[pixelShiftIdx][0], kPixelShiftOffsets[pixelShiftIdx][1]);
    }

    // Gated on the VEHICLE being stationary (user-requested 2026-09-16,
    // "che do tu giam do sang man hinh chi duoc thuc hien khi xe khong
    // chuyen dong sau 3 phut"), not touch-idle time as before — a driver
    // watching the road without touching the screen shouldn't have it dim
    // out from under them while actually driving. gnssMsSinceStationary()
    // itself resets to ~0 the instant real speed or a lost fix is seen (see
    // GNSS.cpp), so this is safe by construction even through a GPS dropout.
    if (cfg.autoDimMin > 0 && !screenDimmed) {
        uint32_t stationaryMs = gnssMsSinceStationary();
        if (stationaryMs > (uint32_t)(cfg.autoDimMin * 60000.0f)) {
            screenDimmed = true;
            applyConfig();
            Serial.println("[uidemo] vehicle stationary -> screen dimmed (burn-in mitigation)");
        }
    }
}

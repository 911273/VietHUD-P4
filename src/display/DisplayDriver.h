#pragma once
#include <lvgl.h>

#ifdef VIETHUD_P4
// ESP32-P4 / JC4880P443C (display/DisplayDriverP4.cpp): ST7701S 480x800
// MIPI-DSI panel. LVGL still renders at the S3 UI's own logical size
// (320x480 portrait / 480x320 landscape per cfg.screenRotation) and the flush
// callback scales it x1.5 (+ rotation) into the DPI framebuffer with the PPA,
// so every layout in ui/ is reused unchanged. The UI only ever asks `gfx`
// for width()/height(), so on this board `gfx` is just that logical size.
struct LogicalDisplay {
    int w = 320, h = 480;
    int width() const { return w; }
    int height() const { return h; }
};
extern LogicalDisplay *gfx;

// Creates the LVGL display at the panel's native resolution (DIRECT mode,
// PSRAM frame, PPA rotate + vsync double buffering) — call after displayBegin().
lv_display_t *displayCreateLvgl();
int displayPhysWidth();  // LVGL resolution: 800x480 landscape / 480x800 portrait
int displayPhysHeight();
const uint16_t *displayFrame(); // the full current LVGL frame (serial 'P' screenshot)
// Flicker diagnostics (serial 'Z'): freeze presentation / steady non-PWM backlight.
void displaySetFrozen(bool on);
void displayTraceFrames(int n); // serial 'F': log the next n presented frames
void displaySetBacklightSteady(bool on);
// Physical panel point (GT911, native portrait) -> LVGL point.
bool displayPanelToLogical(int px, int py, uint16_t *lx, uint16_t *ly);
#else
#include <Arduino_GFX_Library.h>
#endif

// Whole-frame Canvas-based display driver for the JC3248W535 AXS15231B panel.
// See docs/V1.2_hardening_proposal.md "Lỗi phần cứng driver AXS15231B":
// setRotation() on the RAW panel and any non-full-frame draw16bitRGBBitmap()
// on it are both confirmed broken on real hardware — Canvas + always-full-
// frame flush is the required workaround, not a stylistic choice. Do not
// "simplify" this back to a raw panel driver.
//
// Rotation (user-requested 2026-09-15, AppConfig.h's screenRotation) is safe
// to set on the CANVAS specifically, unlike the raw panel above: confirmed
// by reading GFX Library for Arduino/src/canvas/Arduino_Canvas.cpp —
// Arduino_Canvas never forwards setRotation() to the wrapped panel object
// (it uses the plain Arduino_GFX base implementation, pure width/height/
// _rotation bookkeeping), and Canvas::flush() always pushes the *native*
// TFT_RES_W x TFT_RES_H framebuffer to the panel regardless of the Canvas's
// own rotation — the rotation is applied entirely in software while the
// Canvas fills its own RAM framebuffer, never touching the panel's write-
// address-window registers. `gfx` is a plain pointer here (not constructed
// inline) because it needs cfg.screenRotation, which isn't loaded from NVS
// until loadConfigFromNVS() runs partway through setup() — displayBegin()
// constructs it, not this header's static initialization.
#ifndef VIETHUD_P4
extern Arduino_Canvas *gfx;
#endif

// Running totals for the [perf] report in loop() — microseconds spent per
// flush cycle copying into the Canvas RAM buffer (renderUs) vs. the actual
// full-frame push to the panel (flushUs), plus how many flushes occurred.
// Confirmed ~33ms/flush average on real hardware (2026-09-14) — this is the
// measured cost the Giai đoạn 2 esp_lcd port (see the plan doc) is meant to
// fix; keep these counters so any future change can be verified with real
// numbers, not just a feel.
extern uint32_t g_flushUs, g_flushCount, g_renderUs;

void displayBegin();   // gfx->begin(), fillScreen black, logs the post-rotation size
void dispFlushCb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);

// Arduino-ESP32 3.x replaced the channel-based LEDC API with a pin-based one;
// both are supported here so this builds unchanged across core versions.
void backlightBegin();
void backlightWrite(uint32_t duty);

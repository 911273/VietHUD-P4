// ESP32-P4 display driver — Guition JC4880P443C, ST7701S 480x800 MIPI-DSI.
//
// Bring-up recipe (DSI PHY on LDO3 @2.5V, 2 lanes @500Mbps, DPI 34MHz,
// h 12/42/42, v 2/8/166, manual ST7701 init table, reset on GPIO5) is the one
// verified on this exact board by github.com/ultramcu/guition-jc4880p443c-i-w
// (credit elik745i/ESP32-2432S024C-Remote) — the registry esp_lcd_st7701
// component leaves this panel black.
//
// Rendering (v2, 2026-09-29): LVGL renders at the panel's NATIVE resolution
// (800x480 landscape / 480x800 portrait) in DIRECT mode into one PSRAM buffer
// that persists between frames (only dirty areas are redrawn). When a frame
// completes, the PPA rotates the whole buffer into the BACK DPI framebuffer
// (2 of them), and the panel switches to it at the next vblank. The next
// frame waits for "previous framebuffer released" before the PPA touches it
// again — so the panel never scans a half-written frame (the v1 driver wrote
// PPA blocks straight into the one live framebuffer: visible tearing/flicker).
// The UI keeps laying out in its S3 design units; ui/UiScale.h scales them.
#include "DisplayDriver.h"
#include "core/AppConfig.h"
#include "pincfg.h"
#ifdef VIETHUD_P4 // P4-only file; the S3 envs glob this directory too

#include <Arduino.h>
#include <driver/ppa.h>
#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>
#include <freertos/semphr.h>

LogicalDisplay *gfx = nullptr;
uint32_t g_flushUs = 0, g_flushCount = 0, g_renderUs = 0;

static const int kPanelW = LCD_PANEL_W, kPanelH = LCD_PANEL_H; // 480x800 portrait
static int sRot = 0;                 // cfg.screenRotation 0..3
static int sPhysW = 480, sPhysH = 800; // LVGL resolution (landscape = 800x480)

static esp_ldo_channel_handle_t sPhyLdo = nullptr;
static esp_lcd_dsi_bus_handle_t sDsiBus = nullptr;
static esp_lcd_panel_io_handle_t sPanelIo = nullptr;
static esp_lcd_panel_handle_t sPanel = nullptr;
static uint16_t *sFb[2] = {nullptr, nullptr};
static int sFront = 0; // framebuffer index currently scanned out
static ppa_client_handle_t sPpa = nullptr;
static SemaphoreHandle_t sFbFree = nullptr; // given when the previous framebuffer is released
static uint16_t *sLvBuf = nullptr;
static volatile bool sFrozen = false;
static bool sBlSteady = false;
void displaySetFrozen(bool on) { sFrozen = on; }

struct PanelInitCmd {
    uint8_t cmd;
    const uint8_t *data;
    uint8_t len;
    uint16_t delayMs;
};
#define D(...) (const uint8_t[]){__VA_ARGS__}, sizeof((const uint8_t[]){__VA_ARGS__})
static const PanelInitCmd kInit[] = {
    {0xFF, D(0x77, 0x01, 0x00, 0x00, 0x13), 0},
    {0xEF, D(0x08), 0},
    {0xFF, D(0x77, 0x01, 0x00, 0x00, 0x10), 0},
    {0xC0, D(0x63, 0x00), 0},
    {0xC1, D(0x0D, 0x02), 0},
    {0xC2, D(0x10, 0x08), 0},
    {0xCC, D(0x10), 0},
    {0xB0, D(0x80, 0x09, 0x53, 0x0C, 0xD0, 0x07, 0x0C, 0x09, 0x09, 0x28, 0x06, 0xD4, 0x13, 0x69, 0x2B, 0x71), 0},
    {0xB1, D(0x80, 0x94, 0x5A, 0x10, 0xD3, 0x06, 0x0A, 0x08, 0x08, 0x25, 0x03, 0xD3, 0x12, 0x66, 0x6A, 0x0D), 0},
    {0xFF, D(0x77, 0x01, 0x00, 0x00, 0x11), 0},
    {0xB0, D(0x5D), 0},
    {0xB1, D(0x58), 0},
    {0xB2, D(0x87), 0},
    {0xB3, D(0x80), 0},
    {0xB5, D(0x4E), 0},
    {0xB7, D(0x85), 0},
    {0xB8, D(0x21), 0},
    {0xB9, D(0x10, 0x1F), 0},
    {0xBB, D(0x03), 0},
    {0xBC, D(0x00), 0},
    {0xC1, D(0x78), 0},
    {0xC2, D(0x78), 0},
    {0xD0, D(0x88), 0},
    {0xE0, D(0x00, 0x3A, 0x02), 0},
    {0xE1, D(0x04, 0xA0, 0x00, 0xA0, 0x05, 0xA0, 0x00, 0xA0, 0x00, 0x40, 0x40), 0},
    {0xE2, D(0x30, 0x00, 0x40, 0x40, 0x32, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00), 0},
    {0xE3, D(0x00, 0x00, 0x33, 0x33), 0},
    {0xE4, D(0x44, 0x44), 0},
    {0xE5, D(0x09, 0x2E, 0xA0, 0xA0, 0x0B, 0x30, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07, 0x2C, 0xA0, 0xA0), 0},
    {0xE6, D(0x00, 0x00, 0x33, 0x33), 0},
    {0xE7, D(0x44, 0x44), 0},
    {0xE8, D(0x08, 0x2D, 0xA0, 0xA0, 0x0A, 0x2F, 0xA0, 0xA0, 0x04, 0x29, 0xA0, 0xA0, 0x06, 0x2B, 0xA0, 0xA0), 0},
    {0xEB, D(0x00, 0x00, 0x4E, 0x4E, 0x00, 0x00, 0x00), 0},
    {0xEC, D(0x08, 0x01), 0},
    {0xED, D(0xB0, 0x2B, 0x98, 0xA4, 0x56, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x65, 0x4A, 0x89, 0xB2, 0x0B), 0},
    {0xEF, D(0x08, 0x08, 0x08, 0x45, 0x3F, 0x54), 0},
    {0xFF, D(0x77, 0x01, 0x00, 0x00, 0x00), 0},
    {0x11, nullptr, 0, 120}, // SLPOUT
    {0x29, nullptr, 0, 20},  // DISPON
};
#undef D

static bool ok(esp_err_t e, const char *what) {
    if (e != ESP_OK) Serial.printf("[display] %s failed: %s\n", what, esp_err_to_name(e));
    return e == ESP_OK;
}

static bool IRAM_ATTR onFbReleased(esp_lcd_panel_handle_t, esp_lcd_dpi_panel_event_data_t *, void *) {
    BaseType_t woke = pdFALSE;
    xSemaphoreGiveFromISR(sFbFree, &woke);
    return woke == pdTRUE;
}

static bool panelInit() {
    esp_ldo_channel_config_t ldo = {};
    ldo.chan_id = 3;
    ldo.voltage_mv = 2500;
    if (!ok(esp_ldo_acquire_channel(&ldo, &sPhyLdo), "DSI PHY LDO3")) return false;

    esp_lcd_dsi_bus_config_t bus = {};
    bus.bus_id = 0;
    bus.num_data_lanes = 2;
    // NOT MIPI_DSI_PHY_CLK_SRC_DEFAULT: that backward-compat macro still maps
    // to the pre-v3 PLL_F20M source, which rev v3.x silicon doesn't have —
    // the HAL abort()s on it. XTAL is the v3 default.
    bus.phy_clk_src = MIPI_DSI_PHY_PLLREF_CLK_SRC_XTAL;
    // Link/pixel timing = the board's FACTORY firmware, read back from its live
    // registers on 2026-09-29 (esp32p4_board/raw/dsi_health_factory.txt,
    // dsi_clk.txt): 1000 Mbps lanes (VID_HLINE_TIME 2571 lane-byte clocks per
    // 549-pixel line), DPI 240/9 = 26.7 MHz, HFP 1 -> ~50 Hz. The community
    // recipe used before (500 Mbps, 34 MHz, HFP 42, 60 Hz) flickered on this
    // panel even with a frozen frame and a steady backlight.
    bus.lane_bit_rate_mbps = 1000;
    if (!ok(esp_lcd_new_dsi_bus(&bus, &sDsiBus), "DSI bus")) return false;

    esp_lcd_dbi_io_config_t dbi = {};
    dbi.virtual_channel = 0;
    dbi.lcd_cmd_bits = 8;
    dbi.lcd_param_bits = 8;
    if (!ok(esp_lcd_new_panel_io_dbi(sDsiBus, &dbi, &sPanelIo), "DBI io")) return false;

    esp_lcd_dpi_panel_config_t dpi = {};
    dpi.virtual_channel = 0;
    dpi.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi.dpi_clock_freq_mhz = 26; // -> 240 MHz / 9 = 26.67 MHz, as the factory fw
    dpi.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
    dpi.in_color_format = LCD_COLOR_FMT_RGB565;
    dpi.out_color_format = LCD_COLOR_FMT_RGB565;
    dpi.num_fbs = 2;
    dpi.video_timing.h_size = kPanelW;
    dpi.video_timing.v_size = kPanelH;
    dpi.video_timing.hsync_pulse_width = 12;
    dpi.video_timing.hsync_back_porch = 42;
    dpi.video_timing.hsync_front_porch = 1; // factory: H_CFG0 htotal 549 / HLINE 2571 at 1000 Mbps
    dpi.video_timing.vsync_pulse_width = 2;
    dpi.video_timing.vsync_back_porch = 8;
    dpi.video_timing.vsync_front_porch = 166;
    // LP in blanking stays ENABLED (IDF default, as the factory fw:
    // VID_MODE_CFG 0xFF02). disable_lp was tried and caused continuous
    // DPI payload-buffer underflows (INT_ST1 bit19).
    if (!ok(esp_lcd_new_panel_dpi(sDsiBus, &dpi, &sPanel), "DPI panel")) return false;

    pinMode(LCD_RST_PIN, OUTPUT);
    digitalWrite(LCD_RST_PIN, LOW);
    delay(20);
    digitalWrite(LCD_RST_PIN, HIGH);
    delay(120);
    // Same as the board's factory firmware (esp_lcd_st7701 sends these before
    // the vendor table; its boot log read madctl 0x0 / colmod 0x55): pixel
    // format RGB565 to match the 16-bit DSI video stream, normal scan order.
    {
        const uint8_t madctl = 0x00, colmod = 0x55;
        ok(esp_lcd_panel_io_tx_param(sPanelIo, 0x36, &madctl, 1), "MADCTL");
        ok(esp_lcd_panel_io_tx_param(sPanelIo, 0x3A, &colmod, 1), "COLMOD");
    }
    for (const PanelInitCmd &c : kInit) {
        if (!ok(esp_lcd_panel_io_tx_param(sPanelIo, c.cmd, c.data, c.len), "ST7701 init cmd")) return false;
        if (c.delayMs) delay(c.delayMs);
    }
    if (!ok(esp_lcd_panel_init(sPanel), "panel init")) return false;

    void *f0 = nullptr, *f1 = nullptr;
    if (!ok(esp_lcd_dpi_panel_get_frame_buffer(sPanel, 2, &f0, &f1), "get framebuffers") || !f0 || !f1) return false;
    sFb[0] = (uint16_t *)f0;
    sFb[1] = (uint16_t *)f1;
    const size_t fbBytes = (size_t)kPanelW * kPanelH * 2;
    for (auto *fb : sFb) {
        memset(fb, 0, fbBytes);
        esp_cache_msync(fb, fbBytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
    }
    sFront = 0;

    sFbFree = xSemaphoreCreateBinary();
    xSemaphoreGive(sFbFree); // the back buffer starts out free
    esp_lcd_dpi_panel_event_callbacks_t cbs = {};
    cbs.on_frame_buf_complete = onFbReleased;
    if (!ok(esp_lcd_dpi_panel_register_event_callbacks(sPanel, &cbs, nullptr), "DPI callbacks")) return false;

    ppa_client_config_t pc = {};
    pc.oper_type = PPA_OPERATION_SRM;
    pc.max_pending_trans_num = 1;
    if (!ok(ppa_register_client(&pc, &sPpa), "PPA SRM client")) return false;
    return true;
}

void displayBegin() {
    sRot = ((int)cfg.screenRotation) & 3;
    gfx = new LogicalDisplay();
    if (sRot & 1) { // landscape
        sPhysW = kPanelH; sPhysH = kPanelW;
        gfx->w = 533; gfx->h = 320; // design units: x1.5 -> 800x480
    } else {
        sPhysW = kPanelW; sPhysH = kPanelH;
        gfx->w = 320; gfx->h = 533;
    }
    if (!panelInit()) Serial.println("[display] ERROR: MIPI-DSI panel init failed");
    Serial.printf("[display] ST7701S DSI up; LVGL native %dx%d (rotation %d, design %dx%d x1.5), double-buffered\n",
                  sPhysW, sPhysH, sRot, gfx->w, gfx->h);
}

const uint16_t *displayFrame() { return sLvBuf; }
int displayPhysWidth() { return sPhysW; }
int displayPhysHeight() { return sPhysH; }

// Frame trace (serial 'F'): per presented frame, the dirty areas LVGL
// redrew and a sparse content hash — shows whether the dashboard's content
// itself alternates (real flicker) or only moves.
static volatile int sTraceLeft = 0;
static uint32_t sTraceAreas = 0, sTraceArea = 0, sTraceLastMs = 0, sRenderStartUs = 0, sLastWaitUs = 0,
                sLastPpaUs = 0;
void displayTraceFrames(int n) { sTraceLeft = n; sTraceLastMs = millis(); }
static void onRenderStart(lv_event_t *) { sRenderStartUs = micros(); }

void dispFlushCb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    if (sTraceLeft > 0) {
        sTraceAreas++;
        sTraceArea += (uint32_t)(area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    }
    if (sTraceLeft > 0 && lv_display_flush_is_last(disp)) {
        const uint16_t *f = (const uint16_t *)px_map;
        uint32_t h = 2166136261u;
        for (uint32_t i = 0; i < (uint32_t)sPhysW * sPhysH; i += 331) h = (h ^ f[i]) * 16777619u;
        uint32_t now = millis();
        Serial.printf("[frame] dt=%3lums render=%3lums prevWait=%2lums prevPpa=%2lums areas=%lu (%lu%%) hash=%08lx\n",
                      (unsigned long)(now - sTraceLastMs), (unsigned long)((micros() - sRenderStartUs) / 1000),
                      (unsigned long)(sLastWaitUs / 1000), (unsigned long)(sLastPpaUs / 1000),
                      (unsigned long)sTraceAreas,
                      (unsigned long)(sTraceArea * 100 / ((uint32_t)sPhysW * sPhysH)), (unsigned long)h);
        sTraceLastMs = now;
        sTraceAreas = sTraceArea = 0;
        sTraceLeft--;
    }
    // DIRECT mode: px_map is the full persistent frame; act once per frame.
    // sFrozen (flicker diagnostic 'Z', phase B) keeps the panel on the last
    // presented frame.
    if (!lv_display_flush_is_last(disp) || !sPanel || !sPpa || sFrozen) {
        lv_display_flush_ready(disp);
        return;
    }
    uint32_t t0 = micros();
    // Don't touch the back buffer until the panel has let go of it.
    xSemaphoreTake(sFbFree, pdMS_TO_TICKS(60));
    uint32_t t1 = micros();
    sLastWaitUs = t1 - t0;
    int back = sFront ^ 1;
    ppa_srm_oper_config_t op = {};
    op.in.buffer = px_map;
    op.in.pic_w = sPhysW;
    op.in.pic_h = sPhysH;
    op.in.block_w = sPhysW;
    op.in.block_h = sPhysH;
    op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    op.out.buffer = sFb[back];
    op.out.buffer_size = (uint32_t)kPanelW * kPanelH * 2;
    op.out.pic_w = kPanelW;
    op.out.pic_h = kPanelH;
    op.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    // Counter-clockwise angles: 1 = landscape with the UI's top along the
    // panel's right edge (clockwise 90), 3 = the opposite way round.
    static const ppa_srm_rotation_angle_t kAng[4] = {PPA_SRM_ROTATION_ANGLE_0, PPA_SRM_ROTATION_ANGLE_270,
                                                     PPA_SRM_ROTATION_ANGLE_180, PPA_SRM_ROTATION_ANGLE_90};
    op.rotation_angle = kAng[sRot];
    op.scale_x = op.scale_y = 1.0f;
    op.mode = PPA_TRANS_MODE_BLOCKING;
    esp_err_t e = ppa_do_scale_rotate_mirror(sPpa, &op);
    sLastPpaUs = micros() - t1;
    if (e == ESP_OK) {
        // Buffer is one of the panel's own framebuffers: this only switches
        // scan-out to it (at the next frame), no copy.
        esp_lcd_panel_draw_bitmap(sPanel, 0, 0, kPanelW, kPanelH, sFb[back]);
        sFront = back;
        // Drop any "released" signal from a frame that ended BEFORE this swap
        // took effect — the panel still scans the old buffer until the next
        // frame boundary, and a stale token let the next frame write into it
        // mid-scan (the tearing/flicker seen on 2026-09-29).
        xSemaphoreTake(sFbFree, 0);
    } else {
        static uint32_t sErrs = 0;
        if (sErrs++ < 5) Serial.printf("[display] PPA SRM failed: %s\n", esp_err_to_name(e));
        xSemaphoreGive(sFbFree);
    }
    g_renderUs += micros() - t0;
    g_flushCount++;
    lv_display_flush_ready(disp);
}

lv_display_t *displayCreateLvgl() {
    lv_display_t *d = lv_display_create(sPhysW, sPhysH);
    lv_display_set_flush_cb(d, dispFlushCb);
    lv_display_add_event_cb(d, onRenderStart, LV_EVENT_RENDER_START, NULL);
    size_t bytes = (size_t)sPhysW * sPhysH * 2;
    sLvBuf = (uint16_t *)heap_caps_aligned_calloc(64, 1, bytes, MALLOC_CAP_SPIRAM);
    if (!sLvBuf) Serial.println("[display] ERROR: no PSRAM for the LVGL frame buffer");
    lv_display_set_buffers(d, sLvBuf, nullptr, bytes, LV_DISPLAY_RENDER_MODE_DIRECT);
    // Default theme paddings scale with DPI: 1.5x so Settings lists/keyboard
    // match the 1.5x layout.
    lv_display_set_dpi(d, LV_DPI_DEF * 3 / 2);
    return d;
}

// Physical panel point (GT911, native portrait) -> LVGL (native-res) point.
bool displayPanelToLogical(int px, int py, uint16_t *lx, uint16_t *ly) {
    int x, y;
    switch (sRot) {
    case 0: x = px; y = py; break;
    case 2: x = kPanelW - 1 - px; y = kPanelH - 1 - py; break;
    case 1: x = py; y = kPanelW - 1 - px; break;
    default: x = kPanelH - 1 - py; y = px; break;
    }
    if (x < 0 || y < 0 || x >= sPhysW || y >= sPhysH) return false;
    *lx = (uint16_t)x;
    *ly = (uint16_t)y;
    return true;
}

void backlightBegin() { ledcAttach(TFT_BL, 5000, 8); }
void backlightWrite(uint32_t duty) {
    if (sBlSteady) return;
    // 8-bit LEDC: 255 still leaves a 1/256 off-pulse every 200 us on the
    // MP3202 boost enable; full brightness = duty 256, i.e. a steady HIGH.
    ledcWrite(TFT_BL, duty >= 255 ? 256 : duty);
}
// Flicker diagnostic (phase C): backlight as a plain steady-HIGH GPIO, no PWM.
void displaySetBacklightSteady(bool on) {
    if (on == sBlSteady) return;
    sBlSteady = on;
    if (on) {
        ledcDetach(TFT_BL);
        pinMode(TFT_BL, OUTPUT);
        digitalWrite(TFT_BL, HIGH);
    } else {
        ledcAttach(TFT_BL, 5000, 8);
        ledcWrite(TFT_BL, 255);
    }
}
#endif // VIETHUD_P4

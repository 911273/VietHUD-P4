// ESP32-P4 / JC4880P443C touch: GT911 on the I2C bus it shares with the ES8311
// codec (SDA7/SCL8). INT/RST are not wired to usable GPIOs on this board (the
// factory fw also skips the GT9xx address-select reset), so the controller is
// polled at whichever default address answers (0x5D, else 0x14).
//
// Mapping raw -> LVGL pixels: a user calibration (5-point affine least-squares
// fit, touchCalibrationRun(), stored in NVS "tcal" for the rotation it was
// made in) when present; otherwise the nominal panel geometry
// (displayPanelToLogical). The affine form absorbs axis swap/mirror, scale
// and offset in one step, whatever the GT911's own config reports.
#include "TouchTask.h"
#include "display/DisplayDriver.h"
#include "core/AppConfig.h"
#include "pincfg.h"
#ifdef VIETHUD_P4 // P4-only file; the S3 envs glob this directory too
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>

volatile bool gTouchCalRequested = false;

static SemaphoreHandle_t touchMutex;
static TouchPoint sharedPoint;
static uint8_t sAddr = 0;
static volatile int sRawX = 0, sRawY = 0;
static volatile bool sRawPressed = false;
static volatile bool sCalMode = false; // calibration screen owns the touch: LVGL sees no presses

struct TouchCal {
    bool ok = false;
    int rot = -1;
    float a = 0, b = 0, c = 0, d = 0, e = 0, f = 0; // x = a*px + b*py + c ; y = d*px + e*py + f
};
static TouchCal sCal;

TouchPoint touchSnapshot() {
    xSemaphoreTake(touchMutex, portMAX_DELAY);
    TouchPoint copy = sharedPoint;
    xSemaphoreGive(touchMutex);
    return copy;
}

static bool gtRead(uint16_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(sAddr);
    Wire.write(reg >> 8);
    Wire.write(reg & 0xFF);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)sAddr, (int)n) != (int)n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

static void gtWrite8(uint16_t reg, uint8_t v) {
    Wire.beginTransmission(sAddr);
    Wire.write(reg >> 8);
    Wire.write(reg & 0xFF);
    Wire.write(v);
    Wire.endTransmission();
}

static bool gtProbe() {
    for (uint8_t a : {0x5D, 0x14}) {
        sAddr = a;
        uint8_t id[4] = {0};
        if (gtRead(0x8140, id, 4)) {
            uint8_t res[4] = {0};
            gtRead(0x8048, res, 4); // configured X/Y output resolution
            Serial.printf("[touch] GT911 at 0x%02X, product id \"%.4s\", resolution %ux%u\n", a, (const char *)id,
                          res[0] | (res[1] << 8), res[2] | (res[3] << 8));
            return true;
        }
    }
    sAddr = 0;
    return false;
}

static bool mapPoint(int px, int py, uint16_t *lx, uint16_t *ly) {
    if (sCal.ok && sCal.rot == (((int)cfg.screenRotation) & 3)) {
        float x = sCal.a * px + sCal.b * py + sCal.c, y = sCal.d * px + sCal.e * py + sCal.f;
        const int W = displayPhysWidth(), H = displayPhysHeight();
        if (x < -20 || y < -20 || x > W + 20 || y > H + 20) return false;
        *lx = (uint16_t)constrain((int)lroundf(x), 0, W - 1);
        *ly = (uint16_t)constrain((int)lroundf(y), 0, H - 1);
        return true;
    }
    return displayPanelToLogical(px, py, lx, ly);
}

static void touchTaskFn(void *) {
    if (!gtProbe()) Serial.println("[touch] ERROR: GT911 not found on I2C 0x5D/0x14");
    esp_task_wdt_add(NULL);
    uint32_t lastProbeMs = millis();
    for (;;) {
        if (sAddr) {
            uint8_t st = 0;
            if (gtRead(0x814E, &st, 1) && (st & 0x80)) {
                uint8_t n = st & 0x0F;
                uint8_t p[8];
                bool pressed = false;
                uint16_t lx = 0, ly = 0;
                if (n > 0 && gtRead(0x8150, p, 8)) {
                    int px = p[0] | (p[1] << 8), py = p[2] | (p[3] << 8);
                    sRawX = px;
                    sRawY = py;
                    sRawPressed = true;
                    pressed = mapPoint(px, py, &lx, &ly);
                } else {
                    sRawPressed = false;
                }
                gtWrite8(0x814E, 0); // hand the buffer back to the controller
                xSemaphoreTake(touchMutex, portMAX_DELAY);
                sharedPoint.pressed = pressed && !sCalMode;
                if (pressed) {
                    sharedPoint.x = lx;
                    sharedPoint.y = ly;
                }
                xSemaphoreGive(touchMutex);
            }
            // status bit7 clear = no new frame since the last read: keep the
            // previous state (GT911 only raises it when something changed).
        } else if (millis() - lastProbeMs > 3000) {
            lastProbeMs = millis();
            gtProbe();
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void calLoad() {
    Preferences p;
    if (!p.begin("tcal", true)) return;
    sCal.ok = p.getBool("ok", false);
    sCal.rot = p.getInt("rot", -1);
    sCal.a = p.getFloat("a", 0);
    sCal.b = p.getFloat("b", 0);
    sCal.c = p.getFloat("c", 0);
    sCal.d = p.getFloat("d", 0);
    sCal.e = p.getFloat("e", 0);
    sCal.f = p.getFloat("f", 0);
    p.end();
    if (sCal.ok)
        Serial.printf("[touch] calibration loaded (rotation %d): x=%.4f*px%+.4f*py%+.1f  y=%.4f*px%+.4f*py%+.1f\n",
                      sCal.rot, sCal.a, sCal.b, sCal.c, sCal.d, sCal.e, sCal.f);
}

static void calSave() {
    Preferences p;
    if (!p.begin("tcal", false)) return;
    p.putBool("ok", sCal.ok);
    p.putInt("rot", sCal.rot);
    p.putFloat("a", sCal.a);
    p.putFloat("b", sCal.b);
    p.putFloat("c", sCal.c);
    p.putFloat("d", sCal.d);
    p.putFloat("e", sCal.e);
    p.putFloat("f", sCal.f);
    p.end();
}

bool touchCalValid() { return sCal.ok && sCal.rot == (((int)cfg.screenRotation) & 3); }

void touchTaskStart() {
    touchMutex = xSemaphoreCreateMutex();
    calLoad();
    Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);
    Wire.setTimeOut(20);
    xTaskCreatePinnedToCore(touchTaskFn, "touchTask", 3072, NULL, 3, NULL, 0);
}

// ---------------------------------------------------------------------------
// 5-point calibration screen. Blocking, pumps LVGL itself; must run from the
// loop task outside any LVGL callback (main sets gTouchCalRequested).
static uint32_t sPumpLast = 0;
static void pump() {
    uint32_t n = millis();
    lv_tick_inc(n - sPumpLast);
    sPumpLast = n;
    lv_timer_handler();
    esp_task_wdt_reset();
    delay(10);
}

// Solve the 3x3 normal equations for v = k0*px + k1*py + k2 (least squares).
static bool fitAffine(const float *px, const float *py, const float *v, int n, float k[3]) {
    double A[3][4] = {{0}};
    for (int i = 0; i < n; i++) {
        double r[3] = {px[i], py[i], 1.0};
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) A[a][b] += r[a] * r[b];
            A[a][3] += r[a] * v[i];
        }
    }
    for (int c = 0; c < 3; c++) { // Gauss-Jordan with partial pivoting
        int piv = c;
        for (int r = c + 1; r < 3; r++)
            if (fabs(A[r][c]) > fabs(A[piv][c])) piv = r;
        if (fabs(A[piv][c]) < 1e-9) return false;
        for (int j = 0; j < 4; j++) { double t = A[c][j]; A[c][j] = A[piv][j]; A[piv][j] = t; }
        for (int r = 0; r < 3; r++) {
            if (r == c) continue;
            double m = A[r][c] / A[c][c];
            for (int j = 0; j < 4; j++) A[r][j] -= m * A[c][j];
        }
    }
    for (int i = 0; i < 3; i++) k[i] = (float)(A[i][3] / A[i][i]);
    return true;
}

bool touchCalibrationRun(bool firstBoot) {
    const int W = displayPhysWidth(), H = displayPhysHeight();
    const float tx[5] = {0.10f * W, 0.90f * W, 0.90f * W, 0.10f * W, 0.50f * W};
    const float ty[5] = {0.12f * H, 0.12f * H, 0.88f * H, 0.88f * H, 0.50f * H};
    lv_obj_t *prev = lv_screen_active();
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x05080C), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *title = lv_label_create(scr);
    lv_obj_set_style_text_font(title, &lv_font_vn_20, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x3DA5FF), 0);
    lv_label_set_text(title, "HIỆU CHUẨN CẢM ỨNG");
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -60);
    lv_obj_t *msg = lv_label_create(scr);
    lv_obj_set_style_text_color(msg, lv_color_hex(0xE6EDF3), 0);
    lv_obj_align(msg, LV_ALIGN_CENTER, 0, -30);
    // crosshair (real pixel coordinates: plain LVGL calls, not the x1.5 layer)
    lv_obj_t *ring = lv_obj_create(scr);
    (lv_obj_set_size)(ring, 44, 44);
    (lv_obj_set_style_radius)(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, 0);
    (lv_obj_set_style_border_width)(ring, 3, 0);
    lv_obj_set_style_border_color(ring, lv_color_hex(0xFF3B30), 0);
    lv_obj_clear_flag(ring, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t *hl = lv_obj_create(scr), *vl = lv_obj_create(scr);
    for (lv_obj_t *o : {hl, vl}) {
        (lv_obj_set_style_radius)(o, 0, 0);
        (lv_obj_set_style_border_width)(o, 0, 0);
        lv_obj_set_style_bg_color(o, lv_color_white(), 0);
        lv_obj_clear_flag(o, LV_OBJ_FLAG_CLICKABLE);
    }
    (lv_obj_set_size)(hl, 60, 2);
    (lv_obj_set_size)(vl, 2, 60);
    auto place = [&](float x, float y) {
        (lv_obj_set_pos)(ring, (int)x - 22, (int)y - 22);
        (lv_obj_set_pos)(hl, (int)x - 30, (int)y - 1);
        (lv_obj_set_pos)(vl, (int)x - 1, (int)y - 30);
    };
    lv_screen_load(scr);
    sCalMode = true;
    sPumpLast = millis();
    bool done = false;
    for (int attempt = 0; attempt < 3 && !done; attempt++) {
        float rx[5], ry[5];
        bool aborted = false;
        for (int i = 0; i < 5 && !aborted; i++) {
            place(tx[i], ty[i]);
            char b[96];
            snprintf(b, sizeof(b), "Chạm và giữ vào tâm dấu + (%d/5)", i + 1);
            lv_label_set_text(msg, b);
            lv_obj_align(msg, LV_ALIGN_CENTER, 0, -30);
            while (sRawPressed) pump(); // finger from the previous point must lift first
            uint32_t t0 = millis();
            while (!sRawPressed) {
                pump();
                if (firstBoot && millis() - t0 > 30000) { aborted = true; break; }
            }
            if (aborted) break;
            double sx = 0, sy = 0;
            int n = 0;
            uint32_t tp = millis();
            while (sRawPressed || millis() - tp < 150) { // average while held
                if (sRawPressed && millis() - tp > 120) { sx += sRawX; sy += sRawY; n++; }
                pump();
                if (!sRawPressed && n > 0) break;
            }
            if (n == 0) { i--; continue; }
            rx[i] = (float)(sx / n);
            ry[i] = (float)(sy / n);
            lv_obj_set_style_border_color(ring, lv_color_hex(0x34C46A), 0);
            for (int k = 0; k < 15; k++) pump();
            lv_obj_set_style_border_color(ring, lv_color_hex(0xFF3B30), 0);
            Serial.printf("[touchcal] point %d target (%.0f,%.0f) raw (%.1f,%.1f)\n", i + 1, tx[i], ty[i], rx[i], ry[i]);
        }
        if (aborted) {
            Serial.println("[touchcal] no touch within 30 s — skipped (nominal mapping kept)");
            break;
        }
        float kx[3], ky[3];
        if (!fitAffine(rx, ry, tx, 5, kx) || !fitAffine(rx, ry, ty, 5, ky)) {
            lv_label_set_text(msg, "Điểm chạm không hợp lệ, làm lại...");
            for (int k = 0; k < 120; k++) pump();
            continue;
        }
        double err2 = 0, emax = 0;
        for (int i = 0; i < 5; i++) {
            double ex = kx[0] * rx[i] + kx[1] * ry[i] + kx[2] - tx[i];
            double ey = ky[0] * rx[i] + ky[1] * ry[i] + ky[2] - ty[i];
            double e = sqrt(ex * ex + ey * ey);
            err2 += e * e;
            if (e > emax) emax = e;
        }
        double rms = sqrt(err2 / 5);
        Serial.printf("[touchcal] fit rms=%.1fpx max=%.1fpx\n", rms, emax);
        if (emax > 25) {
            lv_label_set_text(msg, "Chưa chính xác, vui lòng chạm lại...");
            for (int k = 0; k < 150; k++) pump();
            continue;
        }
        sCal.ok = true;
        sCal.rot = ((int)cfg.screenRotation) & 3;
        sCal.a = kx[0]; sCal.b = kx[1]; sCal.c = kx[2];
        sCal.d = ky[0]; sCal.e = ky[1]; sCal.f = ky[2];
        calSave();
        char b[80];
        snprintf(b, sizeof(b), "Đã lưu hiệu chuẩn (sai số %.0f px)", rms);
        lv_label_set_text(msg, b);
        lv_obj_add_flag(ring, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(hl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(vl, LV_OBJ_FLAG_HIDDEN);
        for (int k = 0; k < 120; k++) pump();
        done = true;
    }
    while (sRawPressed) pump();
    sCalMode = false;
    lv_screen_load(prev);
    lv_obj_delete(scr);
    return done;
}
#endif // VIETHUD_P4

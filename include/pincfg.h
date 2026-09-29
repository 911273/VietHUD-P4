#pragma once

#ifdef VIETHUD_P4
// Guition JC4880P443C (ESP32-P4 rev v3.2 + ESP32-C6) — pin map read LIVE from
// the board's GPIO/IO_MUX registers while its factory fw ran (2026-09-29,
// esp32p4_board/hw_profile.db) and cross-checked against the board
// schematic (github.com/ultramcu/guition-jc4880p443c-i-w).
// Display is MIPI-DSI (dedicated pins); only reset/backlight are GPIOs.
#define LCD_RST_PIN   5   // ST7701S reset, active low
#define TFT_BL        23  // backlight PWM (LEDC) -> MP3202 boost enable
#define LCD_PANEL_W   480 // native portrait
#define LCD_PANEL_H   800
// Touch GT911 + ES8311 codec share one I2C bus (Wire)
#define TOUCH_SDA     7
#define TOUCH_SCL     8
// Audio: ES8311 over I2S, NS4150 amp enable
#define I2S_MCLK_PIN  13
#define I2S_BCLK_PIN  12
#define I2S_LRCK_PIN  10
#define I2S_DOUT_PIN  9
#define I2S_DIN_PIN   48
#define PA_EN_PIN     11
// microSD: SDMMC slot0 4-bit (IO_MUX pins); TF_VCC comes from on-chip LDO4
#define SD_MMC_CLK_PIN 43
#define SD_MMC_CMD_PIN 44
#define SD_MMC_D0_PIN  39
#define SD_MMC_D1_PIN  40
#define SD_MMC_D2_PIN  41
#define SD_MMC_D3_PIN  42
#define SD_LDO_CHAN    4
// GNSS (u-blox M10N, UART2) — not fitted yet; wire to expansion header JP1:
// module TX -> GPIO30 (JP1 pin 10), module RX <- GPIO31 (JP1 pin 8),
// 3V3 = JP1 pin 1 (or 5V = pin 2), GND = JP1 pin 6.
#define GNSS_RX_PIN 30
#define GNSS_TX_PIN 31
#else

// JC3248W535 (ESP32-S3-N16R8V) pin mapping — confirmed against real-world
// Arduino_GFX/AXS15231B example projects for this exact board, matches
// docs/radar_car_V1.1_spec.md section 19.

// LCD (AXS15231B, QSPI)
#define TFT_BL   1
#define TFT_CS   45
#define TFT_SCK  47
#define TFT_SDA0 21
#define TFT_SDA1 48
#define TFT_SDA2 40
#define TFT_SDA3 39

// Touch (AXS15231B combo controller, I2C)
#define TOUCH_SDA  4
#define TOUCH_SCL  8
#define TOUCH_INT  3
#define TOUCH_ADDR 0x3B

// GNSS (u-blox M10N, UART2) — real wiring as connected and confirmed by the
// user 2026-09-15: GPS module TX -> GNSS_RX_PIN, GPS module RX -> GNSS_TX_PIN
// (TX/RX crossed, as UART always is). This is the OPPOSITE pin assignment
// from the spec section 19 placeholder (which proposed RX=18/TX=17) — the
// real wiring wins; update this if the harness is ever rewired.
#define GNSS_RX_PIN 17 // ESP32 RX, fed by the GPS module's TX
#define GNSS_TX_PIN 18 // ESP32 TX, drives the GPS module's RX

// Onboard microSD/TF slot — CONFIRMED 2026-09-15 from the vendor-adjacent
// demo project for this exact board (refob/Arduino_JC3248W535_LVGL9.4,
// Arduino/mp3_player/pincfg_JC3248W535.h — its TFT_CS/TFT_SCK/TFT_SDA0-3/
// TOUCH_SDA/TOUCH_SCL/TOUCH_INT values match every other entry in this file
// exactly, confirming it's the same board revision, not a different one).
//
// This REPLACES an earlier guess (SD_CS_PIN=10/SD_MOSI_PIN=11/SD_SCK_PIN=12/
// SD_MISO_PIN=13, from a third-party blog describing it as a 4-wire SPI
// interface) that was flat-out the wrong PROTOCOL, not just wrong pin
// numbers: this board wires the TF slot to the ESP32-S3's dedicated
// SD_MMC peripheral in 1-bit mode (3 signals: CLK/CMD/D0, no CS pin exists
// in this mode at all), not general-purpose SPI. That earlier guess's
// SD.h/SPIClass(HSPI-or-FSPI) approach caused two distinct, confirmed
// real-hardware regressions (touch corruption on HSPI, ~5x slower display
// flush on FSPI) — both were downstream of forcing SD traffic onto the same
// SPI2/SPI3 peripherals the display's QSPI bus and Wire/I2C touch driver
// already depend on. SD_MMC's dedicated hardware peripheral shares neither,
// which is the actual fix, not a different pin guess.
#define SD_MMC_CLK_PIN 12
#define SD_MMC_CMD_PIN 11
#define SD_MMC_D0_PIN  13
#endif // VIETHUD_P4

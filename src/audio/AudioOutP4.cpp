#ifdef VIETHUD_P4
#include "AudioOutP4.h"
#include "pincfg.h"
#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s_std.h>

static i2s_chan_handle_t sTx = nullptr;
static uint32_t sRate = 0;
static const uint8_t kEs8311Addr = 0x18;

static bool esWrite(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(kEs8311Addr);
    Wire.write(reg);
    Wire.write(v);
    return Wire.endTransmission() == 0;
}

// Slave mode, MCLK taken from the MCLK pin at 256*fs (same ratio for every
// sample rate, so the clock-divider registers never need rewriting), 16-bit
// Philips I2S in and out. Register values follow Espressif's esp_codec_dev
// es8311 driver (open + 256fs coefficient row + start) — the same driver the
// board's factory firmware used ("ES8311: Work in Slave mode").
static bool es8311Init() {
    static const uint8_t seq[][2] = {
        {0x45, 0x00},
        {0x01, 0x30}, {0x02, 0x00}, {0x03, 0x10}, {0x16, 0x24}, {0x04, 0x10}, {0x05, 0x00},
        {0x0B, 0x00}, {0x0C, 0x00}, {0x10, 0x1F}, {0x11, 0x7F},
        {0x00, 0x80},             // slave, power up
        {0x01, 0x3F},             // all clocks on, MCLK from pin
        {0x06, 0x03},             // SCLK not inverted, bclk_div 4
        {0x07, 0x00}, {0x08, 0xFF}, // LRCK divider (256fs)
        {0x13, 0x10}, {0x1B, 0x0A}, {0x1C, 0x6A},
        {0x09, 0x0C}, {0x0A, 0x0C}, // SDP in/out: I2S, 16-bit
        {0x17, 0xBF}, {0x0E, 0x02}, {0x12, 0x00}, {0x14, 0x1A}, {0x0D, 0x01}, {0x15, 0x40},
        {0x37, 0x08},
        {0x32, 0xBF},             // DAC volume 0 dB (app volume is applied in software)
        {0x31, 0x00},             // DAC unmuted
    };
    for (auto &r : seq)
        if (!esWrite(r[0], r[1])) return false;
    return true;
}

bool p4AudioBegin() {
    pinMode(PA_EN_PIN, OUTPUT);
    digitalWrite(PA_EN_PIN, HIGH);
    // Wire is already up (touchTaskStart runs first); begin() again is a no-op then.
    Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);

    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.auto_clear = true;
    cc.dma_desc_num = 6;
    cc.dma_frame_num = 240;
    if (i2s_new_channel(&cc, &sTx, nullptr) != ESP_OK) {
        Serial.println("[audio] i2s_new_channel failed");
        return false;
    }
    i2s_std_config_t sc = {};
    sc.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000);
    sc.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    sc.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    sc.gpio_cfg.mclk = (gpio_num_t)I2S_MCLK_PIN;
    sc.gpio_cfg.bclk = (gpio_num_t)I2S_BCLK_PIN;
    sc.gpio_cfg.ws = (gpio_num_t)I2S_LRCK_PIN;
    sc.gpio_cfg.dout = (gpio_num_t)I2S_DOUT_PIN;
    sc.gpio_cfg.din = I2S_GPIO_UNUSED;
    if (i2s_channel_init_std_mode(sTx, &sc) != ESP_OK || i2s_channel_enable(sTx) != ESP_OK) {
        Serial.println("[audio] i2s std init failed");
        return false;
    }
    sRate = 16000;
    // MCLK must be running before the codec's clock manager is enabled.
    delay(10);
    if (!es8311Init()) {
        Serial.println("[audio] ES8311 init failed (no ACK at 0x18)");
        return false;
    }
    Serial.println("[audio] ES8311 + I2S std up (MCLK13 BCLK12 WS10 DOUT9, PA GPIO11)");
    return true;
}

bool p4AudioSetRate(uint32_t hz) {
    if (!sTx || hz == 0) return false;
    if (hz == sRate) return true;
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(hz);
    clk.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    i2s_channel_disable(sTx);
    esp_err_t e = i2s_channel_reconfig_std_clock(sTx, &clk);
    i2s_channel_enable(sTx);
    if (e == ESP_OK) sRate = hz;
    return e == ESP_OK;
}

void p4AudioWrite(const int16_t *stereo, size_t frames) {
    if (!sTx) return;
    size_t done = 0;
    i2s_channel_write(sTx, stereo, frames * 4, &done, portMAX_DELAY);
}

bool AudioOutputP4::SetRate(int hz) {
    flush();
    return p4AudioSetRate((uint32_t)hz);
}

bool AudioOutputP4::ConsumeSample(int16_t sample[2]) {
    int16_t l = sample[0], r = channels == 1 ? sample[0] : sample[1];
    buf[n * 2] = Amplify(l);
    buf[n * 2 + 1] = Amplify(r);
    if (++n == kBuf) flush();
    return true;
}

void AudioOutputP4::flush() {
    if (n) p4AudioWrite(buf, n);
    n = 0;
}

bool AudioOutputP4::stop() {
    flush();
    return true;
}
#endif

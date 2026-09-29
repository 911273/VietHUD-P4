#pragma once
#ifdef VIETHUD_P4
// ESP32-P4 / JC4880P443C audio path: ES8311 codec (I2C 0x18 on the shared
// touch bus) fed by one persistent I2S std channel with MCLK = 256*fs, plus
// the NS4150 speaker amp enable. Replaces the S3 build's legacy driver/i2s.h
// tone driver, which IDF 5 refuses to run alongside the new i2s_std driver
// ESP8266Audio 2.x uses — so tones AND mp3 voice share this one channel.
#include <AudioOutput.h>
#include <stdint.h>

bool p4AudioBegin();                               // amp + codec + I2S channel; call once
bool p4AudioSetRate(uint32_t hz);                  // reconfigures the clock only if it changed
void p4AudioWrite(const int16_t *stereo, size_t frames); // blocking write of L/R pairs

// ESP8266Audio sink over the same channel (mp3 voice playback).
class AudioOutputP4 : public AudioOutput {
public:
    bool SetRate(int hz) override;
    bool begin() override { return true; }
    bool ConsumeSample(int16_t sample[2]) override;
    void flush() override;
    bool stop() override;

private:
    static const int kBuf = 128;
    int16_t buf[kBuf * 2];
    int n = 0;
};
#endif

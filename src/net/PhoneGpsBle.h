#pragma once
#ifdef VIETHUD_P4
#include <stddef.h>
#include <stdint.h>

// Phone GPS over BLE (2026-09-29): VietHUD advertises the standard Nordic UART
// Service; a phone writes plain NMEA ($GPRMC/$GPGGA) to its RX characteristic
// and gnss/GNSS.cpp feeds those bytes to the same TinyGPS parser as the UART
// module — but only while the module itself is silent, so a fitted M10N
// always wins. Zero-cost iPhone client: tools/phone-gps/index.html opened in
// the free "Bluefy – Web BLE Browser" app (Safari has no Web Bluetooth).
void phoneGpsBleStart();                      // BLE via the C6 (esp-hosted NimBLE)
size_t phoneGpsRead(uint8_t *buf, size_t max); // drain received NMEA bytes (GNSS task)
bool phoneGpsConnected();
#endif

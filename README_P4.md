# VietHUD-P4

VietHUD (offline GPS speed-limit / camera / traffic-sign warning display) ported
to the **ESP32-P4** — board **Guition JC4880P443C**: ESP32-P4 rev v3.2 +
ESP32-C6 (WiFi 6 / BLE 5 over SDIO), 4.3" ST7701S 480×800 MIPI-DSI, GT911
touch, ES8311 audio, 16 MB flash, 32 MB PSRAM. The ESP32-S3 build
(`env:viethud`) is unchanged and still builds from the same tree.

## What is different on the P4

| Area | P4 implementation |
|---|---|
| Display | Native 800×480 LVGL (DIRECT mode), PPA rotation into two vsync-swapped DPI framebuffers (`src/display/DisplayDriverP4.cpp`). Panel timing = the board's factory firmware (1000 Mbps lanes, 26.67 MHz, ~50 Hz). |
| UI | Same layout code as the S3; `src/ui/UiScale.h` scales design units ×1.5 and swaps in 1.5× fonts (`src/ui/fonts_p4/`). |
| Map | Vector map drawn at native resolution with heading-up rotation baked into the geometry (antialiased, road casings), refreshed ~6–7 Hz. |
| Touch | GT911 + 5-point calibration (auto on first boot, Settings → "Hiệu chuẩn cảm ứng", serial `K`). |
| Audio | ES8311 + one I2S std channel for tones and MP3 voice (`src/audio/AudioOutP4.*`). |
| Data | microSD (4-bit, LDO4 power) or, without a card, the core speedmap in a 13.5 MB read-only FAT partition (`partitions_p4_data.csv`). |
| GPS | M10N on UART2 (GPIO30 RX / GPIO31 TX on JP1), **or phone GPS over BLE**, or the road-following simulator. |

## Phone GPS over BLE (iPhone, zero cost)

1. Install **Bluefy – Web BLE Browser** (free, App Store). Safari has no Web Bluetooth.
2. Open **https://911273.github.io/VietHUD-P4/tools/phone-gps/** in Bluefy.
3. Tap **Kết nối VietHUD**, pick `VietHUD-xxxx`, allow location.

The page sends NMEA ($GPRMC/$GPGGA) over the BLE Nordic UART service; the
firmware feeds it to the same parser as the GPS module (the module wins when
fitted) and shows a **PHONE** badge. Keep Bluefy open on screen while driving
(iOS suspends background web pages). Android: open the same link in Chrome.

## Road simulator

With no GPS source, a simulated car drives the real road network from the
speed map (badge **SIM**). Serial: `R lat lon [speedFactor]` to start anywhere,
`R` to stop.

## Build & flash (Windows, PowerShell)

pioarduino needs PlatformIO Core ≥ 6.2 and must not share a core dir with the
S3 envs (it replaces their pinned platform):

```powershell
$env:PLATFORMIO_CORE_DIR="$HOME\.platformio_p4"
$env:PLATFORMIO_DATA_DIR="<repo>\data_p4"   # data_p4\speedmap = copy of speedmap\
pio run -e viethud_p4 -t upload   --upload-port COM7
pio run -e viethud_p4 -t uploadfs --upload-port COM7   # map data into flash
```

Use board `esp32-p4_r3-evboard` (rev ≥ 3 silicon) and esptool ≥ 5.

## Bench serial commands

`R` road sim · `P` screenshot (raw RGB565) · `F` frame timing trace ·
`Z` flicker diagnostic · `K` touch calibration · `S`/`W` straight/waypoint sims.

Map data © OpenStreetMap contributors (ODbL).

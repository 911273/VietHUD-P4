#ifdef VIETHUD_P4
#include "PhoneGpsBle.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <esp_mac.h>

// Nordic UART Service — generic, so nRF Connect & co. can also drive it.
static const char *kSvcUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *kRxUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"; // phone -> VietHUD (NMEA)
static const char *kTxUuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"; // VietHUD -> phone (status, notify)

static StreamBufferHandle_t sRx = nullptr;
static volatile bool sConnected = false;
static BLECharacteristic *sTx = nullptr;
static uint32_t sRxBytes = 0;

class ServerCb : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        sConnected = true;
        Serial.println("[ble] phone connected");
    }
    void onDisconnect(BLEServer *s) override {
        sConnected = false;
        Serial.println("[ble] phone disconnected -> advertising again");
        s->startAdvertising();
    }
};

class RxCb : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        String v = c->getValue();
        if (!v.length()) return;
        xStreamBufferSend(sRx, v.c_str(), v.length(), 0); // drop on overflow: NMEA resyncs at the next '$'
        sRxBytes += v.length();
        static uint32_t sLastLog = 0;
        if (millis() - sLastLog > 10000) {
            sLastLog = millis();
            Serial.printf("[ble] phone NMEA %lu bytes so far\n", (unsigned long)sRxBytes);
        }
    }
};

void phoneGpsBleStart() {
    sRx = xStreamBufferCreate(2048, 1);
    uint8_t mac[6] = {0};
    char name[20];
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(name, sizeof(name), "VietHUD-%02X%02X", mac[4], mac[5]);
    if (!BLEDevice::init(name)) {
        Serial.println("[ble] init failed (C6 esp-hosted BLE)");
        return;
    }
    BLEServer *srv = BLEDevice::createServer();
    srv->setCallbacks(new ServerCb());
    BLEService *svc = srv->createService(kSvcUuid);
    BLECharacteristic *rx = svc->createCharacteristic(
        kRxUuid, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(new RxCb());
    sTx = svc->createCharacteristic(kTxUuid, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    sTx->setValue("VietHUD ready");
    svc->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(kSvcUuid);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.printf("[ble] advertising \"%s\" (Nordic UART) for phone GPS\n", name);
}

size_t phoneGpsRead(uint8_t *buf, size_t max) {
    if (!sRx) return 0;
    return xStreamBufferReceive(sRx, buf, max, 0);
}

bool phoneGpsConnected() { return sConnected; }
#endif

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

// Nordic UART Service UUIDs
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEService nusService(NUS_SERVICE_UUID);
BLECharacteristic txChar(NUS_TX_UUID, BLERead | BLENotify, 4);

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 8;  // ~125Hz

// 简单的角速度→光标映射，先用浮点验证，后面换定点
float gainX = 8.0f;
float gainY = 8.0f;
float deadbandDPS = 1.5f;  // 1.5 度/秒以下当静止

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== AirMouse NUS ===");

    if (!IMU.begin()) {
        Serial.println("IMU.begin() failed");
        while (1);
    }
    Serial.println("IMU OK");

    if (!BLE.begin()) {
        Serial.println("BLE.begin() failed");
        while (1);
    }

    BLE.setLocalName("AirMouse");
    BLE.setAdvertisedService(nusService);
    nusService.addCharacteristic(txChar);
    BLE.addService(nusService);

    uint8_t zero[4] = {0, 0, 0, 0};
    txChar.writeValue(zero, 4);

    BLE.advertise();
    Serial.println("Advertising as AirMouse");
}

void loop() {
    BLEDevice central = BLE.central();
    if (!central) return;

    Serial.print("Connected to ");
    Serial.println(central.address());

    while (central.connected()) {
        unsigned long now = millis();
        if (now - lastSend < SEND_INTERVAL_MS) continue;
        lastSend = now;

        if (!IMU.gyroscopeAvailable()) continue;

        float gx, gy, gz;
        IMU.readGyroscope(gx, gy, gz);  // dps

        // 角速度直映：gz → dx, gy → dy（轴和方向后面调）
        float wx = gz;
        float wy = gy;

        // 死区
        if (fabsf(wx) < deadbandDPS) wx = 0;
        if (fabsf(wy) < deadbandDPS) wy = 0;

        // 线性增益（先验证，后面换非线性）
        int16_t dx = (int16_t)(wx * gainX);
        int16_t dy = (int16_t)(wy * gainY);

        uint8_t pkt[4];
        pkt[0] = dx & 0xFF;
        pkt[1] = (dx >> 8) & 0xFF;
        pkt[2] = dy & 0xFF;
        pkt[3] = (dy >> 8) & 0xFF;

        txChar.writeValue(pkt, 4);
    }

    Serial.println("Disconnected");
}
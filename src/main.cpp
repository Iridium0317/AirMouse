#include <Arduino.h>
#include "Nano33BleHID.h"

Nano33BleMouse mouse("AirMouse");

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("AirMouse — mbed BLE HID Test");

    Serial.println("[1] calling mouse.initialize()...");
    mouse.initialize();
    Serial.println("[1] done");

    Serial.println("[2] calling RunEventThread()...");
    MbedBleHID_RunEventThread();
    Serial.println("[2] done — if you see this, thread is non-blocking");

    Serial.println("[BLE] Ready.");
}

void loop() {
    Serial.println("loop running");
    if (mouse.connected()) {
        auto *hid = mouse.hid();
        hid->motion(2, 0);
        hid->SendReport();
        Serial.println("Sent dx=2");
    }
    delay(2000);
}
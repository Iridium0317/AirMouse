// GestureLamp — Data Collector Mode
// 在串口输入手势标签 (0-4)，倒计时后录 2 秒 IMU 数据

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>
#include <nrf52840.h>

#define BMI270_I2C_ADDR   0x68
#define BMI270_ACC_X_LSB  0x0C

static float acc_scale = 4.0f / 32768.0f;
static float gyr_scale = 2000.0f / 32768.0f;

typedef struct {
    float ax, ay, az, gx, gy, gz;
} IMUData;

void twim_init(void) {
    NRF_TWIM0->ENABLE = 0;
    NRF_TWIM0->PSEL.SCL = 15;
    NRF_TWIM0->PSEL.SDA = 14;
    NRF_TWIM0->FREQUENCY = 0x06400000;
    NRF_TWIM0->ENABLE = 6;
}

bool twim_read_reg(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    NRF_TWIM0->ADDRESS = addr;
    NRF_TWIM0->TXD.PTR = (uint32_t)&reg;
    NRF_TWIM0->TXD.MAXCNT = 1;
    NRF_TWIM0->RXD.PTR = (uint32_t)buf;
    NRF_TWIM0->RXD.MAXCNT = len;
    NRF_TWIM0->SHORTS = (1 << 7) | (1 << 12);
    NRF_TWIM0->EVENTS_STOPPED = 0;
    NRF_TWIM0->EVENTS_ERROR = 0;
    NRF_TWIM0->TASKS_STARTTX = 1;
    uint32_t timeout = 100000;
    while (!NRF_TWIM0->EVENTS_STOPPED && !NRF_TWIM0->EVENTS_ERROR && --timeout);
    NRF_TWIM0->SHORTS = 0;
    if (NRF_TWIM0->EVENTS_ERROR || timeout == 0) {
        NRF_TWIM0->EVENTS_ERROR = 0;
        NRF_TWIM0->TASKS_STOP = 1;
        return false;
    }
    return true;
}

bool bmi270_read_data(IMUData* data) {
    uint8_t raw[12];
    if (!twim_read_reg(BMI270_I2C_ADDR, BMI270_ACC_X_LSB, raw, 12)) return false;
    data->ax = (int16_t)(raw[0]  | (raw[1]  << 8)) * acc_scale;
    data->ay = (int16_t)(raw[2]  | (raw[3]  << 8)) * acc_scale;
    data->az = (int16_t)(raw[4]  | (raw[5]  << 8)) * acc_scale;
    data->gx = (int16_t)(raw[6]  | (raw[7]  << 8)) * gyr_scale;
    data->gy = (int16_t)(raw[8]  | (raw[9]  << 8)) * gyr_scale;
    data->gz = (int16_t)(raw[10] | (raw[11] << 8)) * gyr_scale;
    return true;
}

// ============ Data Collection Config ============
const int SAMPLE_RATE_HZ = 50;
const unsigned long SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE_HZ;
const int GESTURE_DURATION_MS = 2000;
const int SAMPLES_PER_GESTURE = GESTURE_DURATION_MS / SAMPLE_INTERVAL_MS;

bool recording = false;
int sampleCount = 0;
int currentLabel = -1;
unsigned long lastSampleTime = 0;

void printMenu() {
    Serial.println("\n====== GestureLamp Data Collector ======");
    Serial.println("Input gesture label to start (2 sec each):");
    Serial.println("  0 = Wave");
    Serial.println("  1 = Fist");
    Serial.println("  2 = Rotate");
    Serial.println("  3 = Flick");
    Serial.println("  4 = Idle");
    Serial.println("========================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.print("BMI270 init... ");
    if (!IMU.begin()) { Serial.println("FAILED"); while(1); }
    Serial.println("OK");

    Serial.print("TWIM0 takeover... ");
    twim_init();
    uint8_t id = 0;
    twim_read_reg(BMI270_I2C_ADDR, 0x00, &id, 1);
    if (id != 0x24) { Serial.println("FAILED"); while(1); }
    Serial.println("OK");

    printMenu();
}

void loop() {
    // Wait for user input
    if (!recording && Serial.available()) {
        char c = Serial.read();
        if (c >= '0' && c <= '4') {
            currentLabel = c - '0';
            sampleCount = 0;

            Serial.println("Get ready...");
            delay(500);
            Serial.println("3...");
            delay(500);
            Serial.println("2...");
            delay(500);
            Serial.println("1...");
            delay(500);
            Serial.println("GO!");

            Serial.println("---BEGIN---");
            Serial.println("label,ax,ay,az,gx,gy,gz");

            recording = true;
            lastSampleTime = millis();
        }
    }

    // Recording
    if (recording) {
        unsigned long now = millis();
        if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
            lastSampleTime = now;

            IMUData data;
            if (bmi270_read_data(&data)) {
                Serial.print(currentLabel); Serial.print(",");
                Serial.print(data.ax, 4);  Serial.print(",");
                Serial.print(data.ay, 4);  Serial.print(",");
                Serial.print(data.az, 4);  Serial.print(",");
                Serial.print(data.gx, 4);  Serial.print(",");
                Serial.print(data.gy, 4);  Serial.print(",");
                Serial.println(data.gz, 4);

                sampleCount++;
            }

            if (sampleCount >= SAMPLES_PER_GESTURE) {
                recording = false;
                Serial.println("---END---");
                Serial.print("Done! Gesture ");
                Serial.print(currentLabel);
                Serial.print(", ");
                Serial.print(sampleCount);
                Serial.println(" samples\n");
                printMenu();
            }
        }
    }
}
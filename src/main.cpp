#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>
#include <nrf52840.h>

#define BMI270_ADDR       0x68
#define BMI270_CHIP_ID    0x00
#define BMI270_ACC_X_LSB  0x0C

static float acc_scale = 4.0f / 32768.0f;
static float gyr_scale = 2000.0f / 32768.0f;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} IMUData;

void twim_init(void) {
    NRF_TWIM0->ENABLE = 0;
    NRF_TWIM0->PSEL.SCL = 15;
    NRF_TWIM0->PSEL.SDA = 14;
    NRF_TWIM0->FREQUENCY = 0x06400000;
    NRF_TWIM0->ENABLE = 6;  // TWIM mode
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
    if (!twim_read_reg(BMI270_ADDR, BMI270_ACC_X_LSB, raw, 12)) return false;

    int16_t raw_ax = (int16_t)(raw[0]  | (raw[1]  << 8));
    int16_t raw_ay = (int16_t)(raw[2]  | (raw[3]  << 8));
    int16_t raw_az = (int16_t)(raw[4]  | (raw[5]  << 8));
    int16_t raw_gx = (int16_t)(raw[6]  | (raw[7]  << 8));
    int16_t raw_gy = (int16_t)(raw[8]  | (raw[9]  << 8));
    int16_t raw_gz = (int16_t)(raw[10] | (raw[11] << 8));

    data->ax = raw_ax * acc_scale;
    data->ay = raw_ay * acc_scale;
    data->az = raw_az * acc_scale;
    data->gx = raw_gx * gyr_scale;
    data->gy = raw_gy * gyr_scale;
    data->gz = raw_gz * gyr_scale;
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== GestureLamp Phase 1 ===");

    // Step 1: Library init (uploads 8KB config)
    Serial.print("BMI270 library init... ");
    if (!IMU.begin()) { Serial.println("FAILED"); while(1); }
    Serial.println("OK");

    // Step 2: Take over TWIM0
    Serial.print("Bare-metal TWIM0 takeover... ");
    twim_init();

    uint8_t chip_id = 0;
    twim_read_reg(BMI270_ADDR, BMI270_CHIP_ID, &chip_id, 1);
    Serial.print("Chip ID: 0x");
    Serial.println(chip_id, HEX);

    if (chip_id != 0x24) { Serial.println("FAILED"); while(1); }
    Serial.println("Ready!\n");
    Serial.println("ax,ay,az,gx,gy,gz");
}

unsigned long lastSample = 0;

void loop() {
    if (millis() - lastSample >= 20) {
        lastSample = millis();
        IMUData data;
        if (bmi270_read_data(&data)) {
            Serial.print(data.ax, 4); Serial.print(",");
            Serial.print(data.ay, 4); Serial.print(",");
            Serial.print(data.az, 4); Serial.print(",");
            Serial.print(data.gx, 2); Serial.print(",");
            Serial.print(data.gy, 2); Serial.print(",");
            Serial.println(data.gz, 2);
        }
    }
}
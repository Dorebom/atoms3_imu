// clang-format off
#include "main.hpp"
//#include <M5Unified.h>
#include <M5_IMU_PRO.h>
#include "Imu/MadgwickAHRS.h"
// clang-format on

#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76
#define BMM150_SENSOR_ADDR 0x10

#define NUM_LEDS     1
#define LED_DATA_PIN 35

CRGB leds[NUM_LEDS];

Madgwick filter;

BMI270::BMI270 bmi270;
Adafruit_BMP280 bmp(&Wire);

void setup(void) {
    auto cfg = M5.config();
    cfg.external_imu = true;
    cfg.serial_baudrate = 115200;

    M5_BEGIN(cfg);

    FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(20);

    unsigned status = bmp.begin(BMP280_SENSOR_ADDR);
    if (!status) {
        Serial.println(
            F("Could not find a valid BMP280 sensor, check wiring or "
              "try a different address!"));
        Serial.print("SensorID was: 0x");
        Serial.println(bmp.sensorID(), 16);
        while (1) delay(10);
    }

    bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);
    filter.begin(100);  // 20hz
    filter.setGain(10.0f);
}

float roll = 0;
float pitch = 0;
float yaw = 0;

float temperature = 0;
float pressure = 0;  // Pa
float altitude = 0;  // m

bool is_mode_ascii = true;

void loop(void) {
    M5_UPDATE();

    if (M5.BtnA.wasPressed()) {
        is_mode_ascii = !is_mode_ascii;
    }
    if (is_mode_ascii) {
        leds[0] = CRGB::Red;
    } else {
        leds[0] = CRGB::Green;
    }
    FastLED.show();

    // put your main code here, to run repeatedly:
    float x, y, z;

    if (bmi270.accelerationAvailable() && bmi270.gyroscopeAvailable() &&
        bmi270.magneticFieldAvailable()) {
        float ax, ay, az;
        float gx, gy, gz;
        int16_t mx, my, mz = 0;
        bmi270.readAcceleration(ax, ay, az);
        bmi270.readGyroscope(gx, gy, gz);
        bmi270.readMagneticField(mx, my, mz);
        filter.update(gx, gy, gz, ax, ay, az, (float)mx, (float)my, (float)mz);
        // filter.updateIMU(gx, gy, gz, ax, ay, az);
        roll = filter.getRoll();
        pitch = filter.getPitch();
        yaw = filter.getYaw();

        temperature = bmp.readTemperature();
        pressure = bmp.readPressure();
        altitude = bmp.readAltitude(1013.25);

        if (is_mode_ascii) {
            Serial.printf("roll:%.2f, pitch:%.2f, yaw:%.2f\r\n", roll, pitch,
                          yaw);
            Serial.printf("ax:%.2f, ay:%.2f, az:%.2f\r\n", ax, ay, az);
            Serial.printf("gx:%.2f, gy:%.2f, gz:%.2f\r\n", gx, gy, gz);
            Serial.printf("mx:%.2f, my:%.2f, mz:%.2f\r\n", mx, my, mz);
            Serial.printf("temperature:%.2f C\r\n", temperature);
            Serial.printf("pressure:%.2f hPa\r\n", pressure * 0.01);
            Serial.printf("altitude:%.2f m\r\n", altitude);
        } else {
            // Serial.printf("%.2f,%.2f,%.2f\r\n", roll, pitch, yaw);
        }
    }

    delay(10);
}

// clang-format off
#include "main.hpp"
//#include <M5Unified.h>
#include <M5_IMU_PRO.h>
#include "Imu/MadgwickAHRS.h"
// clang-format on

#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76
#define BMM150_SENSOR_ADDR 0x10

Madgwick filter;

BMI270::BMI270 bmi270;
Adafruit_BMP280 bmp(&Wire);

void setup(void) {
    auto cfg = M5.config();
    cfg.external_imu = true;
    cfg.serial_baudrate = 115200;

    M5_BEGIN(cfg);

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

void loop(void) {
    M5_UPDATE();

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

        Serial.printf("roll:%.2f, pitch:%.2f, yaw:%.2f\r\n", roll, pitch, yaw);

        /*
        Serial.print("accel: \t");
        Serial.print(ax);
        Serial.print('\t');
        Serial.print(ay);
        Serial.print('\t');
        Serial.print(az);
        Serial.println();

        Serial.print("gyro: \t");
        Serial.print(gx);
        Serial.print('\t');
        Serial.print(gy);
        Serial.print('\t');
        Serial.print(gz);
        Serial.println();

        Serial.print("mag: \t");
        Serial.print(mx);
        Serial.print('\t');
        Serial.print(my);
        Serial.print('\t');
        Serial.print(mz);
        Serial.println();
        */
    }

    /*
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); // Adjusted to local forecast!
    Serial.println(" m");
    Serial.println();
    */
    delay(10);
}

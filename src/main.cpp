// clang-format off
#include "main.hpp"
//#include <M5Unified.h>
#include <M5_IMU_PRO.h>
#include "Imu/MadgwickAHRS.h"

#include "system_manager.hpp"
#include "Comm/st_recv_data.hpp"
#include "Comm/st_send_data.hpp"
// clang-format on

#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76
#define BMM150_SENSOR_ADDR 0x10

#define NUM_LEDS     1
#define LED_DATA_PIN 35

//CRGB leds[NUM_LEDS];

Madgwick filter;

BMI270::BMI270 bmi270;
Adafruit_BMP280 bmp(&Wire);

TaskHandle_t taskHandle[2];

SystemManager system_manager;

double timestamp = 0.0;
int periodic_time = 10;  // msec

float roll = 0;
float pitch = 0;
float yaw = 0;

float temperature = 0;
float pressure = 0;  // Pa
float altitude = 0;  // m

bool is_mode_ascii = true;
bool is_send_binary_data = false;

st_send_data send_data;

void task_main(void *pvParameters) {
    for (;;) {
        M5_UPDATE();
        if (M5.BtnA.wasPressed()) {
            is_mode_ascii = !is_mode_ascii;
        }
        if (is_mode_ascii) {
            //leds[0] = CRGB::Red;
            send_data.cmd_type = 0;
        } else {
            //leds[0] = CRGB::Green;
            send_data.cmd_type = 1;
        }
        //FastLED.show();

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
            filter.update(gx, gy, gz, ax, ay, az, (float)mx, (float)my,
                          (float)mz);
            // filter.updateIMU(gx, gy, gz, ax, ay, az);
            roll = filter.getRoll();
            pitch = filter.getPitch();
            yaw = filter.getYaw();

            temperature = bmp.readTemperature();
            pressure = bmp.readPressure();
            altitude = bmp.readAltitude(1013.25);

            if (is_mode_ascii) {
                Serial.printf("roll:%.2f, pitch:%.2f, yaw:%.2f\r\n", roll,
                              pitch, yaw);
                Serial.printf("ax:%.2f, ay:%.2f, az:%.2f\r\n", ax, ay, az);
                Serial.printf("gx:%.2f, gy:%.2f, gz:%.2f\r\n", gx, gy, gz);
                Serial.printf("mx:%.2f, my:%.2f, mz:%.2f\r\n", mx, my, mz);
                Serial.printf("temperature:%.2f C\r\n", temperature);
                Serial.printf("pressure:%.2f hPa\r\n", pressure * 0.01);
                Serial.printf("altitude:%.2f m\r\n", altitude);
            } else {
                send_data.timestamp = (double)millis() / 1000.0;
                send_data.attitude[0] = roll;
                send_data.attitude[1] = pitch;
                send_data.attitude[2] = yaw;
                send_data.angular_velocity[0] = gx;
                send_data.angular_velocity[1] = gy;
                send_data.angular_velocity[2] = gz;
                send_data.linear_acceleration[0] = ax;
                send_data.linear_acceleration[1] = ay;
                send_data.linear_acceleration[2] = az;
                send_data.magnetic_field[0] = mx;
                send_data.magnetic_field[1] = my;
                send_data.magnetic_field[2] = mz;
                send_data.temperature = temperature;
                send_data.pressure = pressure;
                send_data.altitude = altitude;
                send_data.cmd_type = 1;
            }
        }

        /*
        M5.Lcd.clear(BLACK);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.setTextSize(1.5);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("M5 IMU");
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("roll:%.2f, pitch:%.2f, yaw:%.2f\r\n", roll, pitch, yaw);
        //M5.Lcd.println("ascii mode: " + String(is_mode_ascii));
        */
        vTaskDelay(periodic_time / portTICK_PERIOD_MS);
    }
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.println("MAIN DONE");
}

void task_comm(void *pvParameters) {
    SerialManager serial_manager;
    st_recv_data recv_data;
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.println("COMM START");
    for (;;) {
        if (!is_mode_ascii and is_send_binary_data) {
            // send
            system_manager.send_serial(send_data);
        }

        // recv
        if (system_manager.recv_serial(recv_data)) {
            if (recv_data.timestamp - timestamp > 0.0) {
                timestamp = recv_data.timestamp;

                switch (recv_data.cmd_type) {
                    case 0:
                        /* code */
                        break;
                    case 1:
                        is_send_binary_data = true;
                        is_mode_ascii = false;
                        break;
                    case 2:
                        is_send_binary_data = false;
                        filter.setGain(recv_data.cmd_gein);
                        periodic_time = recv_data.cmd_periodic_time;
                        break;
                    case 3:
                        is_send_binary_data = false;
                        is_mode_ascii = true;
                        break;
                    default:
                        break;
                }
            }
        }

        vTaskDelay(periodic_time / portTICK_PERIOD_MS);
    }
}

void setup(void) {
    auto cfg = M5.config();
    cfg.external_imu = true;
    cfg.serial_baudrate = 115200;

    M5_BEGIN(cfg);

    //FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    //FastLED.setBrightness(20);
    int cnt = 0;

    M5.Lcd.setRotation(2);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("M5 IMU");

    delay(1000);

    unsigned status = bmp.begin(BMP280_SENSOR_ADDR);
    if (!status) {
        Serial.println(
            F("Could not find a valid BMP280 sensor, check wiring or "
              "try a different address!"));
        Serial.print("SensorID was: 0x");
        Serial.println(bmp.sensorID(), 16);
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.println("SETUP ---");
        while (1) {
            M5.Lcd.setCursor(0, 40);
            M5.Lcd.println(cnt++);
            delay(10);
            M5.Lcd.clear(BLACK);
        }
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.println("SETUP OK");
    }

    bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);
    filter.begin(100);  // 20hz
    filter.setGain(10.0f);

    M5.Lcd.setCursor(0, 20);
    M5.Lcd.println("SETUP START");
    // xTaskCreate(task_main, "task_main", 4096, NULL, 2, &taskHandle[0]);
    xTaskCreatePinnedToCore(task_main, "task_main", 4096, NULL, 2,
                            &taskHandle[0], 1);
    xTaskCreatePinnedToCore(task_comm, "task_comm", 4096, NULL, 2,
                            &taskHandle[1], 0);
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.println("SETUP DONE");
}

void loop(void) {
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.println("LOOP");

}

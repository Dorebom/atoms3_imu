#include "Comm/Serial/SerialManager.hpp"

// #include <M5AtomS3.h>
//  #include <M5Unified.h>
#include <M5Unified.h>

SerialManager::SerialManager(/* args */) {
}

SerialManager::~SerialManager() {
}

void SerialManager::send(uint8_t *data, int data_size) {
    // encode
    uint8_t encode_buffer[encoding_.getEncodedBufferSize(data_size)];
    encoding_.encode(data, data_size, encode_buffer);
    // send uart
    Serial.write(encode_buffer, sizeof(encode_buffer));
    // USBSerial.write(encode_buffer, sizeof(encode_buffer));
}

void SerialManager::recv(uint8_t *data, int data_size) {
    int encode_buffer_size = encoding_.getEncodedBufferSize(data_size);
    uint8_t encoded_buffer[encode_buffer_size];
    if (Serial.available() == true) {
        Serial.read(encoded_buffer, encode_buffer_size);
        encoding_.decode(encoded_buffer, encode_buffer_size, data);
    } else {
        M5.Lcd.setCursor(0, 80);
        M5.Lcd.printf("recv: %d\n", encode_buffer_size);
    }
    /*
    if (USBSerial.available() == true) {
        USBSerial.read(encoded_buffer, encode_buffer_size);
        encoding_.decode(encoded_buffer, encode_buffer_size, data);
    }
    */
}
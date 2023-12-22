#ifndef SERIAL_MANAGER_HPP
#define SERIAL_MANAGER_HPP

#include "Comm/Serial/encoding.hpp"

class SerialManager
{
private:
    /* data */
    EncodingCobs encoding_;

public:
    SerialManager(/* args */);
    ~SerialManager();
    void send(uint8_t *data, int data_size);
    void recv(uint8_t *data, int data_size);
};


#endif
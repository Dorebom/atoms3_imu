#pragma once

// thread
#include <mutex>

#include "Comm/CRCx/CRCx.h"
#include "Comm/Serial/SerialManager.hpp"
#include "Comm/st_recv_data.hpp"
#include "Comm/st_send_data.hpp"

#define DEFAULT_CRC 0

class SystemManager {
private:
    /* data */
    SerialManager serial_;
    bool check_crc(st_recv_data &data_out);
    uint32_t set_crc(st_send_data &data_in);
    // Mutex
    std::mutex mtx_;

    bool is_success_comm_uart;

public:
    void send_serial(st_send_data &data_in);
    bool recv_serial(st_recv_data &data_out);

    SystemManager(/* args */);
    ~SystemManager();
};

#include "system_manager.hpp"

SystemManager::SystemManager(/* args */) {
}

SystemManager::~SystemManager() {
}

/*
 *   Serial_Comm
 */
void SystemManager::send_serial(st_send_data &data_in) {
    set_crc(data_in);
    serial_.send(reinterpret_cast<uint8_t *>(&data_in), sizeof(st_send_data));
}
bool SystemManager::recv_serial(st_recv_data &data_out) {
    serial_.recv(reinterpret_cast<uint8_t *>(&data_out), sizeof(st_recv_data));
    return check_crc(data_out);
}
bool SystemManager::check_crc(st_recv_data &data_out) {
    uint32_t crc_result;
    uint32_t recv_crc;

    recv_crc = data_out.crc_result;
    data_out.crc_result = DEFAULT_CRC;
    crc_result = crcx::crc32(reinterpret_cast<uint8_t *>(&data_out),
                             sizeof(st_recv_data));
    if (recv_crc != crc_result) {
        is_success_comm_uart = true;
    } else {
        is_success_comm_uart = false;
    }
    return is_success_comm_uart;
}
uint32_t SystemManager::set_crc(st_send_data &data_in) {
    uint32_t crc_result;
    data_in.crc_result = DEFAULT_CRC;
    crc_result = crcx::crc32(reinterpret_cast<uint8_t *>(&data_in),
                             sizeof(st_send_data));
    data_in.crc_result = crc_result;
    return crc_result;
}
/* END Serial_Comm */

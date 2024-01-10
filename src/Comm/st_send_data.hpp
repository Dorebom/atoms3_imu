#pragma once

struct st_send_data
{
    /* data */
    double timestamp;
    float attitude[3];
    float angular_velocity[3];
    float linear_acceleration[3];
    int magnetic_field[3];
    float temperature;
    float pressure;
    float altitude;
    int cmd_type;
    uint32_t crc_result;
};

#pragma once

struct st_recv_data
{
    /* data */
    double timestamp;
    int cmd_type;
    int cmd_periodic_time;  // msec
    float cmd_gein;
    int cmd_data_mode;
    uint32_t crc_result;
};

#pragma once

struct st_system_state
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

    int periodic_time;  // msec
    float gein;
    int data_mode;
};

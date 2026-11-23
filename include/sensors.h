#ifndef SENSORS_H
#define SENSORS_H
#include "sensors.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float temperature_c;
    float humidity_pct;
    uint16_t ambient_light;
    uint16_t proximity;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    bool aht10_ok;
    bool ap3216c_ok;
    bool icm20608_ok;
} StorageSensors_t;

void Sensors_ScanBus(char *out_buf, int buf_size);
void Sensors_InitAll(void);
void Sensors_ReadAll(StorageSensors_t *data);

#endif
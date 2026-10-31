#pragma once
#include "esp_err.h"
#include <stdbool.h>
typedef struct { float ax,ay,az,gx,gy,gz,roll,pitch; } imu_sample_t;
esp_err_t imu_init(void);
esp_err_t imu_calibrate(void);
esp_err_t imu_read(imu_sample_t *out,float dt);
void imu_set_filter_alpha(float alpha);
bool imu_healthy(void);

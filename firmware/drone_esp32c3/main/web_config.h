#pragma once
#include "drone_config.h"
typedef struct { const char *state,*calibration_status;bool imu_ok,radio_ok,calibrating;uint32_t packet_age_ms,loop_us;float roll,pitch,gx,gy,gz,controls[4],motors[4]; } live_status_t;
void web_config_start(drone_config_t *cfg,live_status_t *status,volatile bool *cal_request,volatile bool *armed);

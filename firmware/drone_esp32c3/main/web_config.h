#pragma once
#include "drone_config.h"
typedef struct { const char *state,*calibration_status,*config_status,*save_status,*control_mode;bool imu_ok,radio_ok,calibrating,controls_available;uint32_t packet_age_ms,loop_us,valid_packets,accepted_packets,invalid_packets,lost_packets;uint16_t sequence;float ax,ay,az,roll,pitch,gx,gy,gz,controls[4],motors[4]; } live_status_t;
void web_config_start(drone_config_t *cfg,live_status_t *status,volatile bool *cal_request,volatile bool *armed);

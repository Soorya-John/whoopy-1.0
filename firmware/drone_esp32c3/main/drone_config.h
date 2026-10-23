#pragma once
#include "fc_core.h"
#include "esp_err.h"
#include <stdint.h>
typedef struct { uint32_t version; whoopy_pid_cfg_t roll,pitch,yaw; float max_angle_deg,max_rate_dps,max_motor_output,complementary_alpha; uint16_t failsafe_ms; uint8_t acro; uint8_t motor_map[4]; } drone_config_t;
void config_defaults(drone_config_t *c);
bool config_valid(const drone_config_t *c);
esp_err_t config_load(drone_config_t *c);
esp_err_t config_save(const drone_config_t *c);
esp_err_t config_load_receiver_mode(uint8_t *mode);
esp_err_t config_save_receiver_mode(uint8_t mode);
void config_snapshot(const drone_config_t *source,drone_config_t *destination);
void config_replace(drone_config_t *destination,const drone_config_t *source);

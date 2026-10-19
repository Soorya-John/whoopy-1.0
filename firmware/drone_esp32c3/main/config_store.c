#include "drone_config.h"
#include "nvs.h"
#include <string.h>
#define CFG_VERSION 1u
void config_defaults(drone_config_t*c){*c=(drone_config_t){.version=CFG_VERSION,.roll={.kp=.012f,.ki=.003f,.kd=.0002f,.limit=.25f,.integral_limit=10},.pitch={.kp=.012f,.ki=.003f,.kd=.0002f,.limit=.25f,.integral_limit=10},.yaw={.kp=.008f,.ki=.002f,.kd=0,.limit=.18f,.integral_limit=10},.max_angle_deg=20,.max_rate_dps=180,.max_motor_output=.75f,.complementary_alpha=.98f,.failsafe_ms=250,.acro=0};}
bool config_valid(const drone_config_t*c){return c&&c->version==CFG_VERSION&&whoopy_config_valid(&c->roll,c->max_angle_deg,c->max_rate_dps,c->failsafe_ms)&&whoopy_config_valid(&c->pitch,c->max_angle_deg,c->max_rate_dps,c->failsafe_ms)&&whoopy_config_valid(&c->yaw,c->max_angle_deg,c->max_rate_dps,c->failsafe_ms)&&c->max_motor_output>=.1f&&c->max_motor_output<=1.0f&&c->complementary_alpha>=.90f&&c->complementary_alpha<=.999f&&c->acro<=1;}
esp_err_t config_load(drone_config_t*c){config_defaults(c);nvs_handle_t h;esp_err_t e=nvs_open("whoopy",NVS_READONLY,&h);if(e!=ESP_OK)return e;size_t n=sizeof(*c);e=nvs_get_blob(h,"cfg",c,&n);nvs_close(h);if(e!=ESP_OK||n!=sizeof(*c)||!config_valid(c)){config_defaults(c);return ESP_ERR_INVALID_STATE;}return ESP_OK;}
esp_err_t config_save(const drone_config_t*c){if(!config_valid(c))return ESP_ERR_INVALID_ARG;nvs_handle_t h;esp_err_t e=nvs_open("whoopy",NVS_READWRITE,&h);if(e!=ESP_OK)return e;e=nvs_set_blob(h,"cfg",c,sizeof(*c));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);return e;}

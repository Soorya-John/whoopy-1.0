idf_component_register(SRCS "app_main.c" "radio.c" "imu.c" "motor.c" "config_store.c" "web_config.c" "../../../shared/control/fc_core.c"
 INCLUDE_DIRS "." "../../../shared/control" "../../../shared/protocol"
 REQUIRES driver esp_wifi esp_event esp_netif nvs_flash esp_http_server json)

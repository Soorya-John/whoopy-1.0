#pragma once
#include "esp_err.h"
#include "whoopy_radio_protocol.h"
typedef struct { uint32_t valid_packets,invalid_packets; } radio_stats_t;
typedef enum { RECEIVER_NRF24=0, RECEIVER_ELRS=1 } receiver_mode_t;
esp_err_t radio_init(void);
bool radio_receive(whoopy_control_t *packet);
void radio_set_mode(receiver_mode_t mode);
receiver_mode_t radio_get_mode(void);
bool radio_nrf_detected(void);
bool radio_elrs_detected(void);
void radio_get_stats(radio_stats_t *stats);

#pragma once
#include "esp_err.h"
#include "whoopy_radio_protocol.h"
typedef struct { uint32_t valid_packets,invalid_packets; } radio_stats_t;
esp_err_t radio_init(void);
bool radio_receive(whoopy_control_t *packet);
void radio_get_stats(radio_stats_t *stats);

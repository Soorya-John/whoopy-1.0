#pragma once
#include "esp_err.h"
#include "whoopy_radio_protocol.h"
esp_err_t radio_tx_init(void);
bool radio_tx_send(const whoopy_control_t *packet);
uint32_t radio_tx_successes(void);
uint32_t radio_tx_failures(void);

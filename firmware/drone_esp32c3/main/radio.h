#pragma once
#include "esp_err.h"
#include "whoopy_radio_protocol.h"
esp_err_t radio_init(void);
bool radio_receive(whoopy_control_t *packet);

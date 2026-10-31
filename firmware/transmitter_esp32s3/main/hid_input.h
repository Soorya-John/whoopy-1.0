#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {uint16_t roll,pitch,yaw,throttle;bool arm;uint32_t age_ms;bool valid;} tx_input_t;
void hid_input_init(void);
bool hid_input_get(tx_input_t *out);

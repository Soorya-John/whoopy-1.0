#pragma once
#include <stdbool.h>
void motors_init(void);
void motors_write(const float normalized[4]);
void motors_stop(void);

#ifndef WHOOPY_FC_CORE_H
#define WHOOPY_FC_CORE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { float kp,ki,kd, limit, integral_limit; } whoopy_pid_cfg_t;
typedef struct { float integral, previous, output; bool initialized; } whoopy_pid_state_t;
typedef enum { WHOOPY_BOOT, WHOOPY_INITIALIZING, WHOOPY_DISARMED, WHOOPY_ARMED, WHOOPY_FAILSAFE } whoopy_flight_state_t;
typedef struct { float motor[4]; } whoopy_mix_t;
float whoopy_pid_step(whoopy_pid_state_t *s,const whoopy_pid_cfg_t *c,float setpoint,float measurement,float dt);
void whoopy_mix_x(float throttle,float roll,float pitch,float yaw,whoopy_mix_t *m);
bool whoopy_config_valid(const whoopy_pid_cfg_t *p,float angle_limit,float rate_limit,uint16_t timeout_ms);
whoopy_flight_state_t whoopy_safety_step(whoopy_flight_state_t state,bool init_ok,bool imu_ok,bool link_fresh,bool arm_edge,bool disarm,bool throttle_low);
#endif

#include "fc_core.h"
#include <math.h>

static float clamp(float x, float lo, float hi) {
    return x < lo ? lo : x > hi ? hi : x;
}

float whoopy_pid_step(whoopy_pid_state_t *s, const whoopy_pid_cfg_t *c,
                      float sp, float y, float dt) {
    if (!s || !c || !isfinite(dt) || dt <= 0 || dt > 0.1f) return 0;
    float e = sp - y;
    if (!s->initialized) {
        s->previous = y;
        s->initialized = true;
    }
    float deriv = -(y - s->previous) / dt;
    float trial = clamp(s->integral + e * dt, -c->integral_limit, c->integral_limit);
    float raw = c->kp * e + c->ki * trial + c->kd * deriv;
    if (fabsf(raw) <= c->limit || (raw > c->limit && e < 0) ||
        (raw < -c->limit && e > 0)) s->integral = trial;
    s->previous = y;
    s->output = clamp(c->kp * e + c->ki * s->integral + c->kd * deriv,
                      -c->limit, c->limit);
    return s->output;
}

void whoopy_mix_x(float t, float r, float p, float y, whoopy_mix_t *m) {
    if (!m) return;
    /* Provisional X order: front-left, front-right, rear-right, rear-left.
       Signs assume alternating CW/CCW; verify on the actual frame. */
    m->motor[0] = t + p + r - y;
    m->motor[1] = t + p - r + y;
    m->motor[2] = t - p - r - y;
    m->motor[3] = t - p + r + y;
    for (int i = 0; i < 4; i++) m->motor[i] = clamp(m->motor[i], 0, 1);
}

bool whoopy_config_valid(const whoopy_pid_cfg_t *p, float angle, float rate,
                         uint16_t timeout) {
    if (!p || !isfinite(p->kp) || !isfinite(p->ki) || !isfinite(p->kd) ||
        !isfinite(p->limit) || !isfinite(p->integral_limit)) return false;
    return p->kp >= 0 && p->kp <= 20 && p->ki >= 0 && p->ki <= 10 &&
        p->kd >= 0 && p->kd <= 2 && p->limit > 0 && p->limit <= 1 &&
        p->integral_limit >= 0 && p->integral_limit <= 10 && angle >= 5 &&
        angle <= 45 && rate >= 30 && rate <= 720 && timeout >= 100 &&
        timeout <= 1000;
}

whoopy_flight_state_t whoopy_safety_step(whoopy_flight_state_t s, bool init,
                                        bool imu, bool link, bool arm,
                                        bool disarm, bool low) {
    if (s == WHOOPY_BOOT) return WHOOPY_INITIALIZING;
    if (s == WHOOPY_INITIALIZING)
        return init && imu ? WHOOPY_DISARMED : WHOOPY_FAILSAFE;
    if (s == WHOOPY_FAILSAFE)
        return init && imu && link ? WHOOPY_DISARMED : WHOOPY_FAILSAFE;
    if (s == WHOOPY_DISARMED)
        return arm && init && imu && link && low ? WHOOPY_ARMED : WHOOPY_DISARMED;
    if (s == WHOOPY_ARMED)
        return disarm || !init || !imu || !link
            ? (link && init && imu ? WHOOPY_DISARMED : WHOOPY_FAILSAFE)
            : WHOOPY_ARMED;
    return WHOOPY_FAILSAFE;
}

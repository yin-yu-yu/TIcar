/**
 * @file    filter.c
 * @brief   Signal processing filter implementations
 */

#include "filter.h"
#include <math.h>

/* ========================================================================
 * Public Functions
 * ======================================================================== */

float Filter_LowPass(float input, float *prev, float alpha)
{
    float output = alpha * input + (1.0f - alpha) * (*prev);
    *prev = output;
    return output;
}

float Filter_Complementary(float accel_angle, float gyro_rate, float dt,
                           float alpha, float *fused_angle)
{
    /* High-pass on gyro (integration), low-pass on accel */
    float gyro_angle = *fused_angle + gyro_rate * dt;
    *fused_angle = alpha * accel_angle + (1.0f - alpha) * gyro_angle;
    return *fused_angle;
}

float Filter_RateLimit(float target, float current, float max_rate, float dt)
{
    float max_delta = max_rate * dt;
    float delta = target - current;

    if (delta > max_delta)       return current + max_delta;
    else if (delta < -max_delta) return current - max_delta;
    else                         return target;
}

float Filter_DeadZone(float value, float zone)
{
    if (fabsf(value) < zone) return 0.0f;
    if (value > 0) return value - zone;
    return value + zone;
}

/**
 * @file    filter.h
 * @brief   Signal processing filters
 *
 * Utilities: complementary filter, low-pass filter,
 * moving average, limit/rate limiter.
 */

#ifndef _FILTER_H_
#define _FILTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  First-order low-pass filter
 * @param  input    Current input sample
 * @param  prev     Previous filtered output (updated in place)
 * @param  alpha    Smoothing factor (0~1, smaller = smoother)
 * @return Filtered output
 */
float Filter_LowPass(float input, float *prev, float alpha);

/**
 * @brief  Complementary filter for angle fusion
 * @param  accel_angle   Angle from accelerometer (noisy, no drift)
 * @param  gyro_rate     Angular rate from gyro (deg/s)
 * @param  dt            Time delta (seconds)
 * @param  alpha         Trust factor: accel weight (0~1, typically 0.02)
 * @param  fused_angle   Previous fused angle (updated in place)
 * @return Fused angle
 */
float Filter_Complementary(float accel_angle, float gyro_rate, float dt,
                           float alpha, float *fused_angle);

/**
 * @brief  Rate limiter — limit how fast a value can change
 * @param  target      Desired value
 * @param  current     Current value
 * @param  max_rate    Maximum rate of change per second
 * @param  dt          Time delta (seconds)
 * @return Rate-limited output
 */
float Filter_RateLimit(float target, float current, float max_rate, float dt);

/**
 * @brief  Simple dead-zone filter
 * @param  value    Input value
 * @param  zone     Dead zone threshold (|value| < zone → 0)
 * @return Filtered value
 */
float Filter_DeadZone(float value, float zone);

#ifdef __cplusplus
}
#endif

#endif /* _FILTER_H_ */

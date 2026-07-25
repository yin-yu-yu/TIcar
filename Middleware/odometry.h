/**
 * @file    odometry.h
 * @brief   Wheel odometry — encoder pulses → position & velocity
 *
 * Converts encoder counts to real-world position (x, y, theta)
 * using dead-reckoning with differential drive model.
 */

#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Type Definitions
 * ======================================================================== */

typedef struct {
    float x;       /* Global X position (m)               */
    float y;       /* Global Y position (m)               */
    float theta;   /* Heading angle (rad), (-PI ~ +PI)    */
    float vx;      /* Current forward speed (m/s)         */
    float vz;      /* Current angular speed (rad/s)       */
} Odom_t;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize odometry (reset pose to origin)
 */
void Odom_Init(void);

/**
 * @brief  Update odometry from encoder counts
 * @param  encL     Left encoder pulse count (since last call)
 * @param  encR     Right encoder pulse count (since last call)
 * @param  dt       Time delta (seconds)
 *
 * @note   Call at CONTROL_FREQ_HZ (200Hz) from timer ISR
 * @note   encL/encR are RESET to zero inside encoder ISR after each read
 */
void Odom_Update(int32_t encL, int32_t encR, float dt);

/**
 * @brief  Get current pose and velocity
 * @return Copy of odometry state
 */
Odom_t Odom_GetPose(void);

/**
 * @brief  Reset odometry to origin (0, 0, 0)
 */
void Odom_Reset(void);

/**
 * @brief  Get total distance traveled (m)
 * @return Total travel distance
 */
float Odom_GetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* _ODOMETRY_H_ */

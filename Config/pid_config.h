/**
 * @file    pid_config.h
 * @brief   PID controller parameter presets
 *
 * All PID gains and limits are centralized here.
 * These are DEFAULTS — can be overridden at runtime via Bluetooth APP.
 */

#ifndef _PID_CONFIG_H_
#define _PID_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 1. Velocity PID (speed control loop, 200Hz)
 * ======================================================================== */
#define VELOCITY_KP_DEFAULT     400.0f
#define VELOCITY_KI_DEFAULT     400.0f
#define VELOCITY_KD_DEFAULT     0.0f
#define VELOCITY_OUT_MAX        7800.0f
#define VELOCITY_OUT_MIN       -7800.0f

/* ========================================================================
 * 2. Angle PID (turning / heading control)
 * ======================================================================== */
#define ANGLE_KP_DEFAULT        50.0f
#define ANGLE_KI_DEFAULT        0.0f
#define ANGLE_KD_DEFAULT        5.0f
#define ANGLE_OUT_MAX           3000.0f
#define ANGLE_OUT_MIN          -3000.0f

/* ========================================================================
 * 3. Line-Follow PID (position correction)
 * ======================================================================== */
#define LINE_KP_DEFAULT         30.0f
#define LINE_KI_DEFAULT         0.5f
#define LINE_KD_DEFAULT         10.0f
#define LINE_OUT_MAX            2000.0f
#define LINE_OUT_MIN           -2000.0f

#ifdef __cplusplus
}
#endif

#endif /* _PID_CONFIG_H_ */

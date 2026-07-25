/**
 * @file    motion_control.h
 * @brief   Motion controller — target velocity → motor PWM
 *
 * OWNER:  Team Member
 * STATUS: STUB — to be filled by team member
 *
 * Takes a chassis command (vx, wz), runs PID velocity control,
 * and outputs PWM to motors via Hardware/motor.h.
 *
 * Call flow:
 *   1. MotionControl_SetTarget() — set desired speed (called from state machine)
 *   2. MotionControl_Update()    — compute PID → Motor_SetPWM() (called at 200Hz)
 */

#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_

#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize motion controller
 * @note   Creates PID instances with defaults from pid_config.h
 */
void MotionControl_Init(void);

/**
 * @brief  Set target chassis motion
 * @param  cmd  Desired motion (vx m/s, wz rad/s)
 *
 * Converts chassis command to per-wheel speed targets via
 * inverse kinematics, then feeds each wheel's target to a PID.
 */
void MotionControl_SetTarget(ChassisCmd_t cmd);

/**
 * @brief  Run one control iteration (read encoder → PID → PWM)
 * @note   Call at CONTROL_FREQ_HZ (200Hz) from timer ISR
 *
 * Reads current encoder velocity, computes incremental PI,
 * outputs PWM via Motor_SetPWM().
 */
void MotionControl_Update(void);

/**
 * @brief  Emergency stop — reset PIDs and stop motors
 */
void MotionControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_CONTROL_H_ */

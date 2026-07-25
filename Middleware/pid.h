/**
 * @file    pid.h
 * @brief   Generic PID controller (incremental PI, optional D)
 *
 * Implements incremental PID:
 *   output += Kp*(err - prev_err) + Ki*err + Kd*(err - 2*prev_err + prev_prev_err)
 *
 * Features:
 *   - Output clamping (out_min ~ out_max)
 *   - Integral anti-windup (clamping stops integration)
 *   - PID_Reset() to clear all state
 */

#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Type Definitions
 * ======================================================================== */

typedef struct {
    float Kp;            /* Proportional gain                              */
    float Ki;            /* Integral gain                                  */
    float Kd;            /* Derivative gain (0 = PI only)                  */
    float setpoint;      /* Target value                                   */
    float integral;      /* Accumulated integral error                     */
    float prev_error;    /* Error from previous iteration                  */
    float prev_prev_error; /* Error from two iterations ago (for D term)   */
    float out_min;       /* Minimum output clamp                           */
    float out_max;       /* Maximum output clamp                           */
    float output;        /* Current controller output                      */
} PID_t;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize a PID controller
 * @param  pid   Pointer to PID struct
 * @param  kp    Proportional gain
 * @param  ki    Integral gain
 * @param  kd    Derivative gain (0 = PI only)
 * @param  min   Minimum output value
 * @param  max   Maximum output value
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max);

/**
 * @brief  Set the target (setpoint) value
 * @param  pid       Pointer to PID struct
 * @param  setpoint  Desired target value
 */
void PID_SetSetpoint(PID_t *pid, float setpoint);

/**
 * @brief  Compute one iteration of PID control
 * @param  pid       Pointer to PID struct
 * @param  measured  Current measured value (feedback)
 * @param  dt        Time delta since last call (seconds)
 * @return           Controller output (clamped)
 *
 * @note   Call at fixed frequency (e.g., 200Hz = dt=0.005)
 */
float PID_Compute(PID_t *pid, float measured, float dt);

/**
 * @brief  Reset PID state (clear integral, errors, output)
 * @param  pid  Pointer to PID struct
 */
void PID_Reset(PID_t *pid);

/**
 * @brief  Tune gains at runtime
 * @param  pid  Pointer to PID struct
 * @param  kp   New proportional gain
 * @param  ki   New integral gain
 * @param  kd   New derivative gain
 */
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* _PID_H_ */

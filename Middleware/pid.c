/**
 * @file    pid.c
 * @brief   Generic PID controller implementation
 *
 * Incremental PID with output clamping and anti-windup.
 */

#include "pid.h"

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void PID_Init(PID_t *pid, float kp, float ki, float kd, float min, float max)
{
    pid->Kp            = kp;
    pid->Ki            = ki;
    pid->Kd            = kd;
    pid->setpoint      = 0.0f;
    pid->integral      = 0.0f;
    pid->prev_error    = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->out_min       = min;
    pid->out_max       = max;
    pid->output        = 0.0f;
}

void PID_SetSetpoint(PID_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

float PID_Compute(PID_t *pid, float measured, float dt)
{
    float error = pid->setpoint - measured;

    /* Incremental PID formula:
     * output += Kp*(e(k) - e(k-1)) + Ki*e(k) + Kd*(e(k) - 2*e(k-1) + e(k-2)) */
    float p_term = pid->Kp * (error - pid->prev_error);
    float i_term = pid->Ki * error * dt;
    float d_term = pid->Kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error) / dt;

    pid->output += p_term + i_term + d_term;

    /* Output clamping with anti-windup */
    if (pid->output > pid->out_max) {
        pid->output = pid->out_max;
    } else if (pid->output < pid->out_min) {
        pid->output = pid->out_min;
    }

    /* Store errors for next iteration */
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error      = error;

    return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral      = 0.0f;
    pid->prev_error    = 0.0f;
    pid->prev_prev_error = 0.0f;
    pid->output        = 0.0f;
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    PID_Reset(pid);  /* Reset state when gains change */
}

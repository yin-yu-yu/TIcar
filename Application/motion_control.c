/**
 * @file    motion_control.c
 * @brief   Motion controller — STUB for team member
 *
 * OWNER:  Team Member
 * STATUS: STUB — fill in with real PID and encoder feedback
 *
 * TODO for team member:
 *   1. Create PID_t instances for left & right wheels
 *   2. In MotionControl_SetTarget(): convert ChassisCmd → WheelSpeed via Kinematics_Inverse()
 *   3. In MotionControl_Update(): read encoders, compute PID, call Motor_SetPWM()
 */

#include "motion_control.h"
#include "robot_config.h"
#include "pid_config.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"

/* ========================================================================
 * Module Variables (STUB — team member to complete)
 * ======================================================================== */
static PID_t g_pid_left;
static PID_t g_pid_right;
static float  g_target_left;   /* Target speed (m/s) left  */
static float  g_target_right;  /* Target speed (m/s) right */

extern int Get_Encoder_countA, Get_Encoder_countB;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void MotionControl_Init(void)
{
    /* TODO: Initialize PID controllers with values from pid_config.h
     * PID_Init(&g_pid_left,  VELOCITY_KP_DEFAULT, VELOCITY_KI_DEFAULT, VELOCITY_KD_DEFAULT,
     *          VELOCITY_OUT_MIN, VELOCITY_OUT_MAX);
     * PID_Init(&g_pid_right, ... ); */
    g_target_left  = 0.0f;
    g_target_right = 0.0f;
}

void MotionControl_SetTarget(ChassisCmd_t cmd)
{
    /* TODO: Convert chassis command to wheel targets
     * WheelSpeed_t wheels = Kinematics_Inverse(cmd);
     * PID_SetSetpoint(&g_pid_left,  wheels.left);
     * PID_SetSetpoint(&g_pid_right, wheels.right); */

    WheelSpeed_t wheels = Kinematics_Inverse(cmd);
    g_target_left  = wheels.left;
    g_target_right = wheels.right;

    /* STUB: Direct PWM output (no PID yet)
     * TODO: Replace with proper PID control in MotionControl_Update() */
    int16_t pwm_l = (int16_t)(g_target_left  * PWM_MAX / MAX_LINEAR_SPEED_MPS);
    int16_t pwm_r = (int16_t)(g_target_right * PWM_MAX / MAX_LINEAR_SPEED_MPS);
    Motor_SetPWM(pwm_l, -pwm_r);  /* Note: right motor sign may need flipping */
}

void MotionControl_Update(void)
{
    /* TODO: Real PI velocity control loop
     *
     * // Convert encoder counts to speed (m/s)
     * float speedL = Encoder_GetCountA() * conversion_factor / CONTROL_DT_S;
     * float speedR = Encoder_GetCountB() * conversion_factor / CONTROL_DT_S;
     *
     * // Compute PID
     * float pwm_l = PID_Compute(&g_pid_left,  speedL, CONTROL_DT_S);
     * float pwm_r = PID_Compute(&g_pid_right, speedR, CONTROL_DT_S);
     *
     * // Output
     * Motor_SetPWM((int16_t)pwm_l, (int16_t)pwm_r);
     */

    /* STUB: Nothing yet — Motor_SetPWM called directly in SetTarget */
}

void MotionControl_Stop(void)
{
    PID_Reset(&g_pid_left);
    PID_Reset(&g_pid_right);
    g_target_left  = 0.0f;
    g_target_right = 0.0f;
    Motor_Stop();
}

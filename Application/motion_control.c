/**
 * @file    motion_control.c
 * @brief   Motion controller — chassis command → PID velocity loop → motor PWM
 *
 * OWNER:  Team Member
 * STATUS: COMPLETE — PID velocity control with encoder feedback
 *
 * Data flow (200Hz, from TIMER_0 ISR):
 *   1. SM_Run() → MotionControl_SetTarget(cmd)  — set desired chassis motion
 *   2. MotionControl_Update()                   — read encoders, PID, output PWM
 *
 * Each call to SetTarget stores the target; Update executes the PID loop.
 * This decoupling allows the state machine to set targets inside its handler,
 * while Update runs uniformly after SM_Run() in the ISR.
 */

#include "motion_control.h"
#include "robot_config.h"
#include "pid_config.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
#include "filter.h"       /* for dead-zone */
#include <math.h>
#include <stddef.h>

/* ========================================================================
 * Module Variables
 * ======================================================================== */
static PID_t g_pid_left;
static PID_t g_pid_right;

static float  g_target_left;   /* Target wheel speed (m/s), left  */
static float  g_target_right;  /* Target wheel speed (m/s), right */
static bool   g_target_valid;  /* True when a valid target is set  */

/** Encoder ticks → meters: same conversion factor as odometry */
static float  g_ticks_to_m;

/* ---- Encoder count accumulators (from encoder ISR) ---- */
extern int32_t Get_Encoder_countA;
extern int32_t Get_Encoder_countB;

/* ---- Stop flag (defined in control.c, used across project) ---- */
extern uint8_t Flag_Stop;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void MotionControl_Init(void)
{
    /* Init PID controllers with defaults from pid_config.h */
    PID_Init(&g_pid_left,
             VELOCITY_KP_DEFAULT, VELOCITY_KI_DEFAULT, VELOCITY_KD_DEFAULT,
             VELOCITY_OUT_MIN, VELOCITY_OUT_MAX);
    PID_Init(&g_pid_right,
             VELOCITY_KP_DEFAULT, VELOCITY_KI_DEFAULT, VELOCITY_KD_DEFAULT,
             VELOCITY_OUT_MIN, VELOCITY_OUT_MAX);

    g_target_left  = 0.0f;
    g_target_right = 0.0f;
    g_target_valid = false;

    /* Encoder ticks → meters conversion */
    g_ticks_to_m = WHEEL_PERIMETER_M
                 / (ENCODER_PPR * (float)ENCODER_MULTIPLES * MOTOR_GEAR_RATIO);
}

void MotionControl_SetTarget(ChassisCmd_t cmd)
{
    /* Convert chassis command → wheel speed targets via inverse kinematics */
    WheelSpeed_t wheels = Kinematics_Inverse(cmd);

    g_target_left  = wheels.left;
    g_target_right = wheels.right;
    g_target_valid = true;

    /* Set PID setpoints */
    PID_SetSetpoint(&g_pid_left,  wheels.left);
    PID_SetSetpoint(&g_pid_right, wheels.right);
}

void MotionControl_Update(void)
{
    /* ---- Safety: stop flag halts motors immediately ---- */
    if (Flag_Stop || !g_target_valid) {
        Motor_Stop();
        return;
    }

    /* ---- Read encoder counts (accumulated since last read) ---- */
    int32_t encL = Encoder_GetCountA();
    int32_t encR = Encoder_GetCountB();

    /* Reset for next interval */
    Encoder_Reset();

    /* ---- Convert encoder counts to speed (m/s) ----
     * speed = pulses * meters_per_tick / dt
     *
     * Sign convention:
     *   positive encoder count → forward motion
     *   positive PID output → forward PWM (Motor_SetPWM convention) */
    float dt = CONTROL_DT_S;  /* 0.005s */
    float speedL = (float)encL * g_ticks_to_m / dt;
    float speedR = (float)encR * g_ticks_to_m / dt;

    /* Apply dead-zone to filter encoder noise at very low speeds */
    speedL = Filter_DeadZone(speedL, 0.001f);
    speedR = Filter_DeadZone(speedR, 0.001f);

    /* ---- PID velocity control ----
     * Note: right motor may need sign flip depending on mounting orientation.
     * When both motors are mounted mirror-symmetric, one rotates opposite
     * to achieve the same forward direction. The sign correction belongs
     * in Motor_SetPWM, not here. */
    float pwm_l = PID_Compute(&g_pid_left,  speedL, dt);
    float pwm_r = PID_Compute(&g_pid_right, speedR, dt);

    /* ---- Output to motors ----
     * PWM range: ±VELOCITY_OUT_MAX (±7800)
     * Right motor sign flipped to match mechanical mounting orientation
     * (motors are mounted mirror-symmetric on the chassis) */
    Motor_SetPWM((int16_t)pwm_l, (int16_t)(-pwm_r));
}

void MotionControl_Stop(void)
{
    PID_Reset(&g_pid_left);
    PID_Reset(&g_pid_right);
    g_target_left  = 0.0f;
    g_target_right = 0.0f;
    g_target_valid = false;
    Motor_Stop();
}

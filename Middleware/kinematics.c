/**
 * @file    kinematics.c
 * @brief   Differential drive kinematics implementation
 */

#include "kinematics.h"
#include "robot_config.h"
#include <math.h>

/* ========================================================================
 * Module Variables
 * ======================================================================== */
static float wheel_spacing;     /* Wheel spacing (m) */
static float max_linear;        /* Max linear speed (m/s) */
static float max_angular;       /* Max angular speed (rad/s) */
static float min_turn_radius;   /* Minimum turn radius (m) */

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void Kinematics_Init(void)
{
    wheel_spacing  = WHEEL_SPACING_M;
    max_linear     = MAX_LINEAR_SPEED_MPS;
    max_angular    = MAX_ANGULAR_SPEED_RPS;
    min_turn_radius = TURN_RADIUS_MIN_M;
}

WheelSpeed_t Kinematics_Inverse(ChassisCmd_t cmd)
{
    WheelSpeed_t wheels;

    /* Clamp linear speed */
    if (cmd.vx > max_linear)  cmd.vx = max_linear;
    if (cmd.vx < -max_linear) cmd.vx = -max_linear;

    /* Clamp angular speed */
    if (cmd.wz > max_angular)  cmd.wz = max_angular;
    if (cmd.wz < -max_angular) cmd.wz = -max_angular;

    /* Enforce minimum turn radius for caster-wheel chassis:
     * R = |vx / wz|, must be >= min_turn_radius.
     * If too tight, reduce wz to keep R >= min_turn_radius. */
    if (fabsf(cmd.wz) > 0.001f) {
        float radius = fabsf(cmd.vx / cmd.wz);
        if (radius < min_turn_radius && fabsf(cmd.vx) > 0.001f) {
            cmd.wz = (cmd.wz > 0 ? 1.0f : -1.0f) * fabsf(cmd.vx) / min_turn_radius;
        }
    }

    /* Differential drive inverse kinematics:
     * V_left  = vx - wz * L/2
     * V_right = vx + wz * L/2
     * where L = wheel_spacing */
    float half_diff = cmd.wz * wheel_spacing * 0.5f;
    wheels.left  = cmd.vx - half_diff;
    wheels.right = cmd.vx + half_diff;

    /* Clamp individual wheel speeds */
    if (wheels.left  > max_linear)  wheels.left  = max_linear;
    if (wheels.left  < -max_linear) wheels.left  = -max_linear;
    if (wheels.right > max_linear)  wheels.right = max_linear;
    if (wheels.right < -max_linear) wheels.right = -max_linear;

    return wheels;
}

ChassisCmd_t Kinematics_Forward(WheelSpeed_t wheels)
{
    ChassisCmd_t cmd;
    cmd.vx = (wheels.right + wheels.left) * 0.5f;
    cmd.wz = (wheels.right - wheels.left) / wheel_spacing;
    return cmd;
}

float Kinematics_MinTurnRadius(float vx)
{
    if (fabsf(vx) < 0.001f) return min_turn_radius;
    float r = fabsf(vx) / max_angular;
    return (r < min_turn_radius) ? min_turn_radius : r;
}

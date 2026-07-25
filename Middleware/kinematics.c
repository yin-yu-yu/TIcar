/**
 * @file    kinematics.c
 * @brief   差速驱动运动学实现
 */

#include "kinematics.h"
#include "robot_config.h"
#include <math.h>

/* ========================================================================
 * 模块变量
 * ======================================================================== */
static float wheel_spacing;     /* 轮距 (m) */
static float max_linear;        /* 最大线速度 (m/s) */
static float max_angular;       /* 最大角速度 (rad/s) */
static float min_turn_radius;   /* 最小转弯半径 (m) */

/* ========================================================================
 * 公开函数
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

    /* 限幅线速度 */
    if (cmd.vx > max_linear)  cmd.vx = max_linear;
    if (cmd.vx < -max_linear) cmd.vx = -max_linear;

    /* 限幅角速度 */
    if (cmd.wz > max_angular)  cmd.wz = max_angular;
    if (cmd.wz < -max_angular) cmd.wz = -max_angular;

    /* 万向轮底盘的最小转弯半径约束：
     * R = |vx / wz|，必须 >= min_turn_radius。
     * 若转弯过急，减小 wz 以保持 R >= min_turn_radius。 */
    if (fabsf(cmd.wz) > 0.001f) {
        float radius = fabsf(cmd.vx / cmd.wz);
        if (radius < min_turn_radius && fabsf(cmd.vx) > 0.001f) {
            cmd.wz = (cmd.wz > 0 ? 1.0f : -1.0f) * fabsf(cmd.vx) / min_turn_radius;
        }
    }

    /* 差速驱动逆运动学：
     * V_left  = vx - wz * L/2
     * V_right = vx + wz * L/2
     * 其中 L = wheel_spacing */
    float half_diff = cmd.wz * wheel_spacing * 0.5f;
    wheels.left  = cmd.vx - half_diff;
    wheels.right = cmd.vx + half_diff;

    /* 限幅各轮速度 */
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

/**
 * @file    odometry.c
 * @brief   车轮里程计实现
 */

#include "odometry.h"
#include "kinematics.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 * 模块变量
 * ======================================================================== */
static Odom_t g_odom;
static float  g_distance;       /* 累计行驶距离 (m) */

/** 编码器脉冲 → 米 转换：
 *  脉冲 → 圈数 → 米
 *  米 = 脉冲 / (PPR × 倍频 × 减速比) × 轮周长 */
static float g_ticks_to_m;      /* 1 个编码器脉冲 = ? 米 */

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void Odom_Init(void)
{
    memset(&g_odom, 0, sizeof(Odom_t));
    g_distance = 0.0f;

    /* 转换系数：编码器脉冲 → 车轮行驶距离 (m) */
    g_ticks_to_m = WHEEL_PERIMETER_M
                 / (ENCODER_PPR * ENCODER_MULTIPLES * MOTOR_GEAR_RATIO);
}

void Odom_Update(int32_t encL, int32_t encR, float dt)
{
    /* 将编码器计数转换为行驶距离 (m) */
    float dL = (float)encL * g_ticks_to_m;
    float dR = (float)encR * g_ticks_to_m;

    /* 平均位移和航向变化 */
    float d_center = (dR + dL) * 0.5f;
    float d_theta  = (dR - dL) / WHEEL_SPACING_M;

    /* 更新当前速度 */
    if (dt > 0.0f) {
        g_odom.vx = d_center / dt;
        g_odom.vz = d_theta / dt;
    }

    /* 航位推算位置更新 */
    float half_theta = d_theta * 0.5f;
    g_odom.x     += d_center * cosf(g_odom.theta + half_theta);
    g_odom.y     += d_center * sinf(g_odom.theta + half_theta);
    g_odom.theta += d_theta;

    /* 将 theta 归一化到 [-PI, PI] */
    while (g_odom.theta >  M_PI) g_odom.theta -= 2.0f * M_PI;
    while (g_odom.theta < -M_PI) g_odom.theta += 2.0f * M_PI;

    g_distance += fabsf(d_center);
}

Odom_t Odom_GetPose(void)
{
    return g_odom;
}

void Odom_Reset(void)
{
    memset(&g_odom, 0, sizeof(Odom_t));
    g_distance = 0.0f;
}

float Odom_GetDistance(void)
{
    return g_distance;
}

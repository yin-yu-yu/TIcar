/**
 * @file    odometry.c
 * @brief   Wheel odometry implementation
 */

#include "odometry.h"
#include "kinematics.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

/* ========================================================================
 * Module Variables
 * ======================================================================== */
static Odom_t g_odom;
static float  g_distance;       /* Total distance traveled (m) */

/** Encoder ticks to meters conversion:
 *  pulses → revolutions → meters
 *  meters = pulses / (PPR * Multiples * GearRatio) * WheelPerimeter */
static float g_ticks_to_m;      /* 1 encoder tick = ? meters */

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void Odom_Init(void)
{
    memset(&g_odom, 0, sizeof(Odom_t));
    g_distance = 0.0f;

    /* Conversion factor: encoder ticks → wheel distance (m) */
    g_ticks_to_m = WHEEL_PERIMETER_M
                 / (ENCODER_PPR * ENCODER_MULTIPLES * MOTOR_GEAR_RATIO);
}

void Odom_Update(int32_t encL, int32_t encR, float dt)
{
    /* Convert encoder counts to distance traveled (m) */
    float dL = (float)encL * g_ticks_to_m;
    float dR = (float)encR * g_ticks_to_m;

    /* Average distance and heading change */
    float d_center = (dR + dL) * 0.5f;
    float d_theta  = (dR - dL) / WHEEL_SPACING_M;

    /* Update current velocity */
    if (dt > 0.0f) {
        g_odom.vx = d_center / dt;
        g_odom.vz = d_theta / dt;
    }

    /* Dead-reckoning position update */
    float half_theta = d_theta * 0.5f;
    g_odom.x     += d_center * cosf(g_odom.theta + half_theta);
    g_odom.y     += d_center * sinf(g_odom.theta + half_theta);
    g_odom.theta += d_theta;

    /* Normalize theta to [-PI, PI] */
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

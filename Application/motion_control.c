/**
 * @file    motion_control.c
 * @brief   运动控制器 — 底盘指令 → PID 速度环 → 电机 PWM
 *
 * 负责人：团队成员
 * 状态：已完成 — 带编码器反馈的 PID 速度控制
 *
 * 数据流（200Hz，来自 TIMER_0 ISR）：
 *   1. SM_Run() → MotionControl_SetTarget(cmd)  — 设置期望底盘运动
 *   2. MotionControl_Update()                   — 读取编码器、PID、输出 PWM
 *
 * SetTarget 存储目标值；Update 执行 PID 循环。
 * 这种解耦允许状态机在其处理函数中设置目标，
 * 而 Update 在 ISR 中的 SM_Run() 之后统一运行。
 */

#include "motion_control.h"
#include "robot_config.h"
#include "pid_config.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "kinematics.h"
#include "filter.h"       /* 用于死区滤波 */
#include <math.h>
#include <stddef.h>

/* ========================================================================
 * 模块变量
 * ======================================================================== */
static PID_t g_pid_left;
static PID_t g_pid_right;

static float  g_target_left;   /* 左轮目标速度 (m/s) */
static float  g_target_right;  /* 右轮目标速度 (m/s) */
static bool   g_target_valid;  /* 有效目标已设置时为真 */

/** 编码器脉冲 → 米：与里程计使用相同的转换系数 */
static float  g_ticks_to_m;

/* ---- 编码器计数累加器（来自编码器 ISR）---- */
extern volatile int32_t Get_Encoder_countA;
extern volatile int32_t Get_Encoder_countB;

/* ---- 停止标志（在 control.c 中定义，全项目使用）---- */
extern uint8_t Flag_Stop;

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void MotionControl_Init(void)
{
    /* 使用 pid_config.h 中的默认值初始化 PID 控制器 */
    PID_Init(&g_pid_left,
             VELOCITY_KP_DEFAULT, VELOCITY_KI_DEFAULT, VELOCITY_KD_DEFAULT,
             VELOCITY_OUT_MIN, VELOCITY_OUT_MAX);
    PID_Init(&g_pid_right,
             VELOCITY_KP_DEFAULT, VELOCITY_KI_DEFAULT, VELOCITY_KD_DEFAULT,
             VELOCITY_OUT_MIN, VELOCITY_OUT_MAX);

    g_target_left  = 0.0f;
    g_target_right = 0.0f;
    g_target_valid = false;

    /* 编码器脉冲 → 米 转换 */
    g_ticks_to_m = WHEEL_PERIMETER_M
                 / (ENCODER_PPR * (float)ENCODER_MULTIPLES * MOTOR_GEAR_RATIO);
}

void MotionControl_SetTarget(ChassisCmd_t cmd)
{
    /* 通过逆运动学将底盘指令 → 车轮速度目标 */
    WheelSpeed_t wheels = Kinematics_Inverse(cmd);

    g_target_left  = wheels.left;
    g_target_right = wheels.right;
    g_target_valid = true;

    /* 设置 PID 目标值 */
    PID_SetSetpoint(&g_pid_left,  wheels.left);
    PID_SetSetpoint(&g_pid_right, wheels.right);
}

void MotionControl_Update(void)
{
    /* ---- 安全保护：停止标志立即停止电机 ---- */
    if (Flag_Stop || !g_target_valid) {
        Motor_Stop();
        return;
    }

    /* ---- 读取编码器计数（自上次读取以来的累计值）---- */
    int32_t encL = Encoder_GetCountA();
    int32_t encR = Encoder_GetCountB();

    /* 复位以用于下一间隔 */
    Encoder_Reset();

    /* ---- 将编码器计数转换为速度 (m/s) ----
     * speed = pulses * meters_per_tick / dt
     *
     * 符号约定：
     *   正编码器计数 → 前进运动
     *   正 PID 输出 → 正向 PWM（Motor_SetPWM 约定） */
    float dt = CONTROL_DT_S;  /* 0.005s */
    float speedL = (float)encL * g_ticks_to_m / dt;
    float speedR = (float)encR * g_ticks_to_m / dt;

    /* 对极低速下的编码器噪声应用死区滤波 */
    speedL = Filter_DeadZone(speedL, 0.001f);
    speedR = Filter_DeadZone(speedR, 0.001f);

    /* ---- PID 速度控制 ----
     * 注意：右电机可能需要根据安装方向对符号取反。
     * 当两个电机镜像对称安装时，一个电机需要反向旋转
     * 才能实现相同的向前方向。符号修正应在 Motor_SetPWM
     * 中处理，而非此处。 */
    float pwm_l = PID_Compute(&g_pid_left,  speedL, dt);
    float pwm_r = PID_Compute(&g_pid_right, speedR, dt);

    /* ---- 输出到电机 ----
     * PWM 范围：±VELOCITY_OUT_MAX (±7800)
     * 右电机符号取反以匹配机械安装方向
     * （电机在底盘上镜像对称安装） */
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

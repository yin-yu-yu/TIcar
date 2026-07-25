/**
 * @file    motion_control.c
 * @brief   运动控制器 — 待团队成员填充的 STUB
 *
 * 负责人：团队成员
 * 状态：STUB — 请填充真实的 PID 和编码器反馈逻辑
 *
 * 团队成员 TODO:
 *   1. 为左右轮创建 PID_t 实例
 *   2. 在 MotionControl_SetTarget() 中：通过 Kinematics_Inverse() 将 ChassisCmd → WheelSpeed
 *   3. 在 MotionControl_Update() 中：读取编码器、计算 PID、调用 Motor_SetPWM()
 */

#include "motion_control.h"
#include "robot_config.h"
#include "pid_config.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"

/* ========================================================================
 * 模块变量（STUB — 待团队成员完成）
 * ======================================================================== */
static PID_t g_pid_left;
static PID_t g_pid_right;
static float  g_target_left;   /* 左轮目标速度 (m/s) */
static float  g_target_right;  /* 右轮目标速度 (m/s) */

extern int Get_Encoder_countA, Get_Encoder_countB;

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void MotionControl_Init(void)
{
    /* TODO: 使用 pid_config.h 中的值初始化 PID 控制器
     * PID_Init(&g_pid_left,  VELOCITY_KP_DEFAULT, VELOCITY_KI_DEFAULT, VELOCITY_KD_DEFAULT,
     *          VELOCITY_OUT_MIN, VELOCITY_OUT_MAX);
     * PID_Init(&g_pid_right, ... ); */
    g_target_left  = 0.0f;
    g_target_right = 0.0f;
}

void MotionControl_SetTarget(ChassisCmd_t cmd)
{
    /* TODO: 将底盘指令转换为车轮目标速度
     * WheelSpeed_t wheels = Kinematics_Inverse(cmd);
     * PID_SetSetpoint(&g_pid_left,  wheels.left);
     * PID_SetSetpoint(&g_pid_right, wheels.right); */

    WheelSpeed_t wheels = Kinematics_Inverse(cmd);
    g_target_left  = wheels.left;
    g_target_right = wheels.right;

    /* STUB: 直接 PWM 输出（尚未使用 PID）
     * TODO: 在 MotionControl_Update() 中替换为正确的 PID 控制 */
    int16_t pwm_l = (int16_t)(g_target_left  * PWM_MAX / MAX_LINEAR_SPEED_MPS);
    int16_t pwm_r = (int16_t)(g_target_right * PWM_MAX / MAX_LINEAR_SPEED_MPS);
    Motor_SetPWM(pwm_l, -pwm_r);  /* 注意：右电机符号可能需要取反 */
}

void MotionControl_Update(void)
{
    /* TODO: 真正的 PI 速度控制循环
     *
     * // 将编码器计数值转换为速度 (m/s)
     * float speedL = Encoder_GetCountA() * conversion_factor / CONTROL_DT_S;
     * float speedR = Encoder_GetCountB() * conversion_factor / CONTROL_DT_S;
     *
     * // 计算 PID
     * float pwm_l = PID_Compute(&g_pid_left,  speedL, CONTROL_DT_S);
     * float pwm_r = PID_Compute(&g_pid_right, speedR, CONTROL_DT_S);
     *
     * // 输出
     * Motor_SetPWM((int16_t)pwm_l, (int16_t)pwm_r);
     */

    /* STUB: 暂未实现 — Motor_SetPWM 在 SetTarget 中直接调用 */
}

void MotionControl_Stop(void)
{
    PID_Reset(&g_pid_left);
    PID_Reset(&g_pid_right);
    g_target_left  = 0.0f;
    g_target_right = 0.0f;
    Motor_Stop();
}

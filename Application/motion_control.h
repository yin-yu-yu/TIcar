/**
 * @file    motion_control.h
 * @brief   运动控制器：将目标速度转换为电机 PWM
 *
 * OWNER:  Team Member
 * STATUS: STUB — to be filled by team member
 *
 * 接收底盘命令（vx、wz），执行 PID 速度控制，
 * 并通过 Hardware/motor.h 输出电机 PWM。
 *
 * Call flow:
 *   1. MotionControl_SetTarget() — set desired speed (called from state machine)
 *   2. MotionControl_Update()    — compute PID → Motor_SetPWM() (called at 200Hz)
 */

#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_

#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  初始化运动控制器
 * @note   使用 pid_config.h 中的默认参数创建 PID 实例
 */
void MotionControl_Init(void);

/**
 * @brief  设置目标底盘运动
 * @param  cmd  期望运动（vx：m/s，wz：rad/s）
 *
 * 通过逆运动学将底盘命令转换为各轮速度目标，
 * 再将每个车轮的目标值送入 PID。
 */
void MotionControl_SetTarget(ChassisCmd_t cmd);

/**
 * @brief  执行一次控制迭代（读取编码器、PID 计算、输出 PWM）
 * @note   Call at CONTROL_FREQ_HZ (200Hz) from timer ISR
 *
 * 读取当前编码器速度，计算增量式 PI，
 * 并通过 Motor_SetPWM() 输出 PWM。
 */
void MotionControl_Update(void);

/**
 * @brief  紧急停止：重置 PID 并停止电机
 */
void MotionControl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_CONTROL_H_ */

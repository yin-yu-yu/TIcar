/**
 * @file    pid_config.h
 * @brief   PID 控制器参数预设
 *
 * 所有 PID 增益与限幅参数均集中在此处。
 * 以下为默认值，可在运行时通过蓝牙 APP 覆盖。
 */

#ifndef _PID_CONFIG_H_
#define _PID_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 1. 速度 PID（200Hz 速度控制环）
 * ======================================================================== */
#define VELOCITY_KP_DEFAULT     400.0f
#define VELOCITY_KI_DEFAULT     400.0f
#define VELOCITY_KD_DEFAULT     0.0f
#define VELOCITY_OUT_MAX        7800.0f
#define VELOCITY_OUT_MIN       -7800.0f

/* ========================================================================
 * 2. 角度 PID（转向/航向控制）
 * ======================================================================== */
#define ANGLE_KP_DEFAULT        50.0f
#define ANGLE_KI_DEFAULT        0.0f
#define ANGLE_KD_DEFAULT        5.0f
#define ANGLE_OUT_MAX           3000.0f
#define ANGLE_OUT_MIN          -3000.0f

/* ========================================================================
 * 3. 循迹 PID（位置修正）
 * ======================================================================== */
#define LINE_KP_DEFAULT         30.0f
#define LINE_KI_DEFAULT         0.5f
#define LINE_KD_DEFAULT         10.0f
#define LINE_OUT_MAX            2000.0f
#define LINE_OUT_MIN           -2000.0f

#ifdef __cplusplus
}
#endif

#endif /* _PID_CONFIG_H_ */

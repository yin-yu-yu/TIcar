/**
 * @file    ir_track.h
 * @brief   红外循迹传感器驱动（四通道）
 *
 * 读取四个红外反射传感器以检测线路位置。
 * 包含循迹状态机。
 *
 * 硬件：四个数字红外传感器（DH1 ~ DH4）
 */

#ifndef _IR_TRACK_H_
#define _IR_TRACK_H_

#include <stdint.h>
#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 传感器状态位定义
 * ======================================================================== */
/* Bit mapping: DH1=bit3, DH2=bit2, DH3=bit1, DH4=bit0
 * Black line detected → bit = 1, White ground → bit = 0 */
#define IR_SENSOR_ALL_WHITE     0x00    /* 路口/全白               */
#define IR_SENSOR_ALL_BLACK     0x0F    /* 全黑/丢线               */
#define IR_SENSOR_STRAIGHT      0x09    /* DH1 + DH4（在线上）     */

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  读取原始四通道传感器状态
 * @return 4 位数值：bit3=DH1，bit2=DH2，bit1=DH3，bit0=DH4
 */
uint8_t IR_GetSensorState(void);

/**
 * @brief  获取线路位置误差（供 PID 控制使用）
 * @return 位置误差（mm；0 为居中，负值偏左，正值偏右）
 */
float IR_GetPositionError(void);

/**
 * @brief  执行一次循迹状态机迭代
 * @note   应在定时器中断中以 CONTROL_FREQ_HZ（200Hz）调用，
 *         函数内部会更新电机目标速度
 */
void IR_LineDetect_Update(void);

/**
 * @brief  设置循迹基础巡航速度
 * @param  speed_mmps  速度（mm/s）
 */
void IR_SetBaseSpeed(float speed_mmps);

/**
 * @brief  获取当前期望转向差值
 * @return 转向差角
 */
float IR_GetTurnDiff(void);

/**
 * @brief  获取上次 IR_LineDetect_Update() 计算的底盘命令
 * @return 可直接传给 MotionControl_SetTarget() 的 ChassisCmd_t（vx、wz）
 *
 * This bridges the Hardware→Application gap cleanly:
 * IR_LineDetect_Update() computes the command internally,
 * and the Application layer retrieves it via this accessor.
 */
ChassisCmd_t IR_GetLineFollowCmd(void);

/* ---- 外部全局状态（供调试/显示使用） ---- */
extern uint32_t ir_dh1_state;
extern uint32_t ir_dh2_state;
extern uint32_t ir_dh3_state;
extern uint32_t ir_dh4_state;

/* ---- 外部循迹参数（可由蓝牙 APP 调整） ---- */
extern float Turn90Angle;
extern float TurnMaxAngle;
extern float TurnMidAngle;
extern float TurnMinAngle;
extern float BaseSpeed;
extern float ForwardLimit;

#ifdef __cplusplus
}
#endif

#endif /* _IR_TRACK_H_ */

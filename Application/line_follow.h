/**
 * @file    line_follow.h
 * @brief   循迹控制策略
 *
 * OWNER:  Team Member
 * STATUS: STUB — to be filled by team member
 *
 * 从 Hardware/ir_track.h 读取红外传感器状态，计算速度和转向修正，
 * 并将目标值传入 MotionControl。
 */

#ifndef _LINE_FOLLOW_H_
#define _LINE_FOLLOW_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  初始化循迹参数
 */
void LineFollow_Init(void);

/**
 * @brief  根据传感器状态计算速度修正量
 * @param  sensor_state  4 位红外传感器读数（0x0 ~ 0xF）
 * @return 速度修正值（m/s，正值表示右转）
 *
 * @note   函数内部也会更新 MotionControl 的目标速度
 */
float LineFollow_ComputeCorrection(uint8_t sensor_state);

/**
 * @brief  设置基础巡航速度
 * @param  speed_mps  速度（m/s）
 */
void LineFollow_SetBaseSpeed(float speed_mps);

#ifdef __cplusplus
}
#endif

#endif /* _LINE_FOLLOW_H_ */

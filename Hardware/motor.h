/**
 * @file    motor.h
 * @brief   电机 PWM 与方向控制驱动
 *
 * 通过 TIMA1 PWM（CCP0/CCP1）控制两个直流电机，
 * 并使用 GPIO（AIN1/AIN2、BIN1/BIN2）控制方向。
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  设置左右电机 PWM 与方向
 * @param  pwmL   左电机 PWM（-8000 ~ +8000，正值为前进）
 * @param  pwmR   右电机 PWM（-8000 ~ +8000，正值为前进）
 */
void Motor_SetPWM(int16_t pwmL, int16_t pwmR);

/* ---- 向后兼容的别名 ---- */
void Set_PWM(int16_t pwmL, int16_t pwmR);

/**
 * @brief  停止两个电机（滑行）
 */
void Motor_Stop(void);

/**
 * @brief  制动两个电机（通过 GPIO 短接制动）
 */
void Motor_Brake(void);

#ifdef __cplusplus
}
#endif

#endif /* _MOTOR_H_ */

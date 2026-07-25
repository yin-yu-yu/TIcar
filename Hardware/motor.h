/**
 * @file    motor.h
 * @brief   Motor PWM + direction control driver
 *
 * Controls two DC motors via TIMA1 PWM (CCP0/CCP1)
 * with GPIO direction control (AIN1/AIN2, BIN1/BIN2).
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Set left and right motor PWM + direction
 * @param  pwmL   Left motor PWM (-8000 ~ +8000, positive = forward)
 * @param  pwmR   Right motor PWM (-8000 ~ +8000, positive = forward)
 */
void Motor_SetPWM(int16_t pwmL, int16_t pwmR);

/* ---- Backward-compatible alias ---- */
void Set_PWM(int16_t pwmL, int16_t pwmR);

/**
 * @brief  Stop both motors (coast)
 */
void Motor_Stop(void);

/**
 * @brief  Brake both motors (short brake via GPIO)
 */
void Motor_Brake(void);

#ifdef __cplusplus
}
#endif

#endif /* _MOTOR_H_ */

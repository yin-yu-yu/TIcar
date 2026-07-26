/**
 * @file    motor.c
 * @brief   Motor PWM + direction control driver
 *
 * Controls two DC motors via TIMA1 PWM (CCP0=left, CCP1=right)
 * with GPIO direction control (AIN1/AIN2, BIN1/BIN2).
 *
 * H-bridge truth table (per channel):
 *   Forward:  IN1=LOW,  IN2=HIGH  → positive PWM value
 *   Backward: IN1=HIGH, IN2=LOW   → negative PWM value
 *   Coast:    IN1=LOW,  IN2=LOW   → PWM=0 (Motor_Stop)
 *   Brake:    IN1=HIGH, IN2=HIGH  → PWM=0 (Motor_Brake)
 *
 * If the car does NOT move straight with Motor_SetPWM(+2000,+2000):
 *   → swap AIN1/AIN2 polarity below for the motor that runs backward.
 *
 * Original hardware: WHEELTEC C07A
 * All rights reserved
 ***********************************************/

#include "motor.h"
#include "board.h"
#include "robot_config.h"

/* ---- Internal helper: set one motor channel ---- */
static void Motor_SetChannel(uint32_t port, uint32_t pin1, uint32_t pin2,
                             int16_t pwm, uint32_t ccIdx)
{
    uint32_t duty = (uint32_t)ABS(pwm);

    /* Dead-zone compensation: if PWM is non-zero but too weak to move
     * the motor, boost it to the minimum effective value.  Set
     * PWM_DEAD_ZONE to 0 in Config/robot_config.h if not needed. */
    if (PWM_DEAD_ZONE > 0 && duty > 0 && duty < (uint32_t)PWM_DEAD_ZONE) {
        duty = (uint32_t)PWM_DEAD_ZONE;
    }

    /* Clamp to PWM_MAX */
    if (duty > (uint32_t)PWM_MAX) {
        duty = (uint32_t)PWM_MAX;
    }

    if (pwm > 0) {
        DL_GPIO_setPins(port, pin2);
        DL_GPIO_clearPins(port, pin1);
    } else if (pwm < 0) {
        DL_GPIO_setPins(port, pin1);
        DL_GPIO_clearPins(port, pin2);
    } else {
        /* pwm == 0 → coast (both IN pins LOW) */
        DL_GPIO_clearPins(port, pin1 | pin2);
        duty = 0;
    }

    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty, ccIdx);
}

/* ---- Public API ---- */

void Set_PWM(int16_t pwmL, int16_t pwmR)
{
    Motor_SetChannel(AIN_PORT, AIN_AIN1_PIN, AIN_AIN2_PIN,
                     pwmL, GPIO_PWM_0_C0_IDX);
    Motor_SetChannel(BIN_PORT, BIN_BIN1_PIN, BIN_BIN2_PIN,
                     pwmR, GPIO_PWM_0_C1_IDX);
}

void Motor_SetPWM(int16_t pwmL, int16_t pwmR)
{
    Set_PWM(pwmL, pwmR);
}

void Motor_Stop(void)
{
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
}

void Motor_Brake(void)
{
    DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
}

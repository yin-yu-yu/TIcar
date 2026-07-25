/**
 * @file    ir_track.h
 * @brief   Infrared line tracking sensor driver (4-channel)
 *
 * Reads 4 IR reflectance sensors to detect line position.
 * Includes the line-following state machine.
 *
 * Hardware: 4 digital IR sensors (DH1 ~ DH4)
 */

#ifndef _IR_TRACK_H_
#define _IR_TRACK_H_

#include <stdint.h>
#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Sensor State Bit Definitions
 * ======================================================================== */
/* Bit mapping: DH1=bit3, DH2=bit2, DH3=bit1, DH4=bit0
 * Black line detected → bit = 1, White ground → bit = 0 */
#define IR_SENSOR_ALL_WHITE     0x00    /* Cross / all white       */
#define IR_SENSOR_ALL_BLACK     0x0F    /* All black / lost        */
#define IR_SENSOR_STRAIGHT      0x09    /* DH1 + DH4 (on line)     */

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Read raw 4-channel sensor state
 * @return 4-bit value: bit3=DH1, bit2=DH2, bit1=DH3, bit0=DH4
 */
uint8_t IR_GetSensorState(void);

/**
 * @brief  Get line position error (for PID control)
 * @return Position error in mm (0 = centered, negative = left, positive = right)
 */
float IR_GetPositionError(void);

/**
 * @brief  Run one iteration of the line-following state machine
 * @note   Call this at CONTROL_FREQ_HZ (200Hz) from timer ISR
 *         Updates target motor speeds internally
 */
void IR_LineDetect_Update(void);

/**
 * @brief  Set base cruising speed for line following
 * @param  speed_mmps  Speed in mm/s
 */
void IR_SetBaseSpeed(float speed_mmps);

/**
 * @brief  Get current desired turn differential
 * @return Turn differential angle
 */
float IR_GetTurnDiff(void);

/**
 * @brief  Get the chassis command computed by the last IR_LineDetect_Update()
 * @return ChassisCmd_t (vx, wz) ready for MotionControl_SetTarget()
 *
 * This bridges the Hardware→Application gap cleanly:
 * IR_LineDetect_Update() computes the command internally,
 * and the Application layer retrieves it via this accessor.
 */
ChassisCmd_t IR_GetLineFollowCmd(void);

/* ---- Extern global state (debug/display use) ---- */
extern uint32_t ir_dh1_state;
extern uint32_t ir_dh2_state;
extern uint32_t ir_dh3_state;
extern uint32_t ir_dh4_state;

/* ---- Extern line-follow parameters (BT APP tunable) ---- */
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

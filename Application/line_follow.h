/**
 * @file    line_follow.h
 * @brief   Line-following control strategy
 *
 * OWNER:  Team Member
 * STATUS: STUB — to be filled by team member
 *
 * Reads IR sensor state from Hardware/ir_track.h, computes
 * speed/turn corrections, and feeds target to MotionControl.
 */

#ifndef _LINE_FOLLOW_H_
#define _LINE_FOLLOW_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize line-following parameters
 */
void LineFollow_Init(void);

/**
 * @brief  Compute speed correction from sensor state
 * @param  sensor_state  4-bit IR sensor reading (0x0 ~ 0xF)
 * @return Speed correction value (m/s, positive = turn right)
 *
 * @note   Also updates MotionControl target speeds internally
 */
float LineFollow_ComputeCorrection(uint8_t sensor_state);

/**
 * @brief  Set base cruising speed
 * @param  speed_mps  Speed in m/s
 */
void LineFollow_SetBaseSpeed(float speed_mps);

#ifdef __cplusplus
}
#endif

#endif /* _LINE_FOLLOW_H_ */

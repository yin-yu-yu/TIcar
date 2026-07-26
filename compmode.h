/**
 * @file    compmode.h
 * @brief   Competition mode — autonomous / remote-control operation loop
 *
 * Include and call CompMode_Run() from main() when CHASSIS_DEBUG is NOT defined.
 * This is the robot's normal operating mode: state machine + BT protocol + OLED.
 */

#ifndef _COMPMODE_H_
#define _COMPMODE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Run the competition-mode sequence and enter the main control loop.
 * @note   Does not return — contains an infinite while(1) loop.
 *
 * Sequence:
 *   1. Show "COMP MODE" on OLED for 3s
 *   2. Enter competition loop — BT protocol handling, status reporting,
 *      OLED display refresh, battery monitoring
 */
void CompMode_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* _COMPMODE_H_ */

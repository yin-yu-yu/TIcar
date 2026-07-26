/**
 * @file    debugmode.h
 * @brief   Debug mode — interface verification and team member test area
 *
 * Include and call DebugMode_Run() from main() when CHASSIS_DEBUG is defined.
 */

#ifndef _DEBUGMODE_H_
#define _DEBUGMODE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Run the debug-mode sequence and enter the debug main loop.
 * @note   Does not return — contains an infinite while(1) loop.
 *
 * Sequence:
 *   1. Show "DEBUG MODE" on OLED for 3s
 *   2. Run 13 interface verification tests (motor, encoder, IR, battery,
 *      PID, kinematics, odometry, motion control, line follow, BT protocol,
 *      debug scope, filter, MPU6050)
 *   3. Enter debug loop — team members add test code here
 */
void DebugMode_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* _DEBUGMODE_H_ */

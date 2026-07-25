/**
 * @file    state_machine.h
 * @brief   Robot state machine — top-level behavior controller
 *
 * OWNER:  Team Lead (队长)
 * STATUS: Active development
 *
 * Manages robot operational states and transitions:
 *
 *   STATE_INIT ──(self-test ok)──▶ STATE_IDLE
 *                                      │
 *                     ┌────────────────┼────────────────┐
 *                     ▼                ▼                 ▼
 *              STATE_RC_DRIVE   STATE_LINE_FOLLOW   STATE_LOW_BATTERY
 *                     │                │                 │
 *                     └────────────────┼─────────────────┘
 *                                      ▼
 *                                STATE_IDLE
 *
 * STATE_ERROR is reachable from any state on critical failure.
 *
 * CALL FLOW: SM_Run() is called at 200Hz from TIMER_0 ISR.
 */

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Type Definitions
 * ======================================================================== */

typedef enum {
    STATE_INIT = 0,         /* Power-on self-test / initialization    */
    STATE_IDLE,             /* Standby, waiting for command            */
    STATE_RC_DRIVE,         /* Remote control driving (BT APP)         */
    STATE_LINE_FOLLOW,      /* Autonomous line following (IR sensors)  */
    STATE_LOW_BATTERY,      /* Low battery warning (LED+OLED alert)    */
    STATE_ERROR,            /* Critical fault (motor stop, alert)      */
    STATE_COUNT
} RobotState_t;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize state machine
 * @note   Call once at startup after hardware init
 */
void SM_Init(void);

/**
 * @brief  Run one iteration of the state machine
 * @note   Call at CONTROL_FREQ_HZ (200Hz) from TIMER_0 ISR
 *
 * Reads sensor inputs, evaluates transitions, and calls
 * the appropriate state handler.
 */
void SM_Run(void);

/**
 * @brief  Request a state transition
 * @param  next  Target state
 * @note   Rejected if transition is not valid from current state
 */
void SM_Transition(RobotState_t next);

/**
 * @brief  Get current robot state
 * @return Current state enum value
 */
RobotState_t SM_GetState(void);

/**
 * @brief  Get human-readable state name (for OLED display)
 * @param  state  State value
 * @return String name (e.g., "IDLE", "RC_DRIVE")
 */
const char* SM_GetStateName(RobotState_t state);

/**
 * @brief  Check if robot is in a safe-to-move state
 * @return true if motors may be active
 */
bool SM_IsMoving(void);

/**
 * @brief  Report battery voltage level to state machine
 * @param  voltage  Battery voltage (V)
 * @note   Called from main loop; SM evaluates low-battery condition
 */
void SM_ReportBattery(float voltage);

#ifdef __cplusplus
}
#endif

#endif /* _STATE_MACHINE_H_ */

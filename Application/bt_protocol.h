/**
 * @file    bt_protocol.h
 * @brief   Bluetooth communication protocol (WheelTec APP compatible)
 *
 * OWNER:  Team Member
 * STATUS: STUB — to be filled by team member
 *
 * Parses UART1 DMA buffer for APP commands.
 * Protocol: WheelTec binary format with 0x7B/0x7D framing.
 *
 * Commands supported:
 *   - Direction control (joystick, steering)
 *   - Speed adjustment
 *   - PID parameter setting
 *   - Mode switching
 */

#ifndef _BT_PROTOCOL_H_
#define _BT_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include "state_machine.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Type Definitions
 * ======================================================================== */

typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_MOVE,       /* Joystick direction (1~8)      */
    BT_CMD_STEER,      /* Steering rotation (L/R)        */
    BT_CMD_SPEED_UP,   /* Increase speed                 */
    BT_CMD_SPEED_DOWN, /* Decrease speed                 */
    BT_CMD_PID_SET,    /* Set PID parameter              */
    BT_CMD_PID_QUERY,  /* Request PID report             */
    BT_CMD_MODE_SWITCH,/* Switch run mode                */
} BT_CmdType_t;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

/**
 * @brief  Initialize Bluetooth protocol parser
 */
void BT_Protocol_Init(void);

/**
 * @brief  Process Bluetooth receive buffer
 * @note   Call from main loop or periodically
 *
 * Reads DMA buffer, extracts complete frames, and
 * dispatches commands.
 */
void BT_Protocol_Handler(void);

/**
 * @brief  Send robot status over Bluetooth
 * @param  state     Current robot state
 * @param  batt_v    Battery voltage (V)
 * @param  speed_l   Left wheel speed (mm/s)
 * @param  speed_r   Right wheel speed (mm/s)
 */
void BT_Protocol_SendStatus(RobotState_t state, float batt_v,
                            float speed_l, float speed_r);

/**
 * @brief  Send PID parameter report (for APP display)
 */
void BT_Protocol_SendPID(void);

#ifdef __cplusplus
}
#endif

#endif /* _BT_PROTOCOL_H_ */

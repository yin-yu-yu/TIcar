/**
 * @file    bt_protocol.c
 * @brief   Bluetooth protocol parser — STUB for team member
 *
 * OWNER:  Team Member
 * STATUS: STUB — fill in with real protocol handling
 *
 * TODO: The existing BT protocol logic is in Control/uart_callback.c
 *       (functions: BTBufferHandler, bt_control, BT_DAMConfig).
 *       Migrate and enhance into this module.
 */

#include "bt_protocol.h"
#include "uart_bt.h"
#include "robot_config.h"
#include "pid_config.h"

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void BT_Protocol_Init(void)
{
    /* TODO: Initialize BT DMA reception
     * BT_DMAConfig(); */
}

void BT_Protocol_Handler(void)
{
    /* TODO: Port existing BTBufferHandler() logic from Control/uart_callback.c
     *
     * Pseudo-code:
     *   uint8_t recvsize = BT_PACKET_SIZE - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);
     *   if (recvsize != lastSize) {
     *       // New data arrived, process byte-by-byte
     *       for (i = lastHandled; i < recvsize; i++) {
     *           bt_control(gBTPacket[i]);  // Parse each command byte
     *       }
     *   }
     */
}

void BT_Protocol_SendStatus(RobotState_t state, float batt_v,
                            float speed_l, float speed_r)
{
    /* TODO: Format and send status packet over Bluetooth
     *
     * Existing format: printf("{A%d:%d:%d:%d}$", speedLeft, speedRight, battPct, 0);
     * See Control/show.c APP_Show() for reference. */
    (void)state;
    (void)batt_v;
    (void)speed_l;
    (void)speed_r;
}

void BT_Protocol_SendPID(void)
{
    /* TODO: Send current PID values to APP
     *
     * Existing format:
     * printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d:$}",
     *   (int)Velocity_KP, (int)Velocity_KI, (int)BaseSpeed,
     *   (int)Turn90Angle, (int)TurnMaxAngle, (int)TurnMidAngle,
     *   (int)TurnMinAngle, (int)ForwardLimit, 0); */
}

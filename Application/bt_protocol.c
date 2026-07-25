/**
 * @file    bt_protocol.c
 * @brief   蓝牙协议解析器 — 待团队成员填充的 STUB
 *
 * 负责人：团队成员
 * 状态：STUB — 请填充真实的协议处理逻辑
 *
 * TODO: 现有的蓝牙协议逻辑在 Control/uart_callback.c 中
 *       （函数：BTBufferHandler, bt_control, BT_DAMConfig）。
 *       请迁移并增强到本模块中。
 */

#include "bt_protocol.h"
#include "uart_bt.h"
#include "robot_config.h"
#include "pid_config.h"

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void BT_Protocol_Init(void)
{
    /* TODO: 初始化蓝牙 DMA 接收
     * BT_DMAConfig(); */
}

void BT_Protocol_Handler(void)
{
    /* TODO: 从 Control/uart_callback.c 移植现有的 BTBufferHandler() 逻辑
     *
     * 伪代码：
     *   uint8_t recvsize = BT_PACKET_SIZE - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);
     *   if (recvsize != lastSize) {
     *       // 有新数据到达，逐字节处理
     *       for (i = lastHandled; i < recvsize; i++) {
     *           bt_control(gBTPacket[i]);  // 解析每个命令字节
     *       }
     *   }
     */
}

void BT_Protocol_SendStatus(RobotState_t state, float batt_v,
                            float speed_l, float speed_r)
{
    /* TODO: 格式化并通过蓝牙发送状态数据包
     *
     * 现有格式：printf("{A%d:%d:%d:%d}$", speedLeft, speedRight, battPct, 0);
     * 参考 Control/show.c 中的 APP_Show()。 */
    (void)state;
    (void)batt_v;
    (void)speed_l;
    (void)speed_r;
}

void BT_Protocol_SendPID(void)
{
    /* TODO: 发送当前 PID 参数到 APP
     *
     * 现有格式：
     * printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d:$}",
     *   (int)Velocity_KP, (int)Velocity_KI, (int)BaseSpeed,
     *   (int)Turn90Angle, (int)TurnMaxAngle, (int)TurnMidAngle,
     *   (int)TurnMinAngle, (int)ForwardLimit, 0); */
}

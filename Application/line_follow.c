/**
 * @file    line_follow.c
 * @brief   循线策略 — 待团队成员填充的 STUB
 *
 * 负责人：团队成员
 * 状态：STUB — 请填充真实的循线算法
 *
 * TODO: 现有的红外循线状态机在 Hardware/IR_Module.c 中
 *       （函数 IRDM_line_inspection()）。请迁移并增强到本模块中。
 */

#include "line_follow.h"
#include "ir_track.h"
#include "motion_control.h"
#include "kinematics.h"
#include "robot_config.h"

/* ========================================================================
 * 模块变量
 * ======================================================================== */
static float g_base_speed = 0.15f;  /* m/s，默认 150mm/s */

extern float turn_diff;
extern int Flag_Stop;

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void LineFollow_Init(void)
{
    g_base_speed = 0.15f;
}

float LineFollow_ComputeCorrection(uint8_t sensor_state)
{
    /* TODO: 实现循线 PID 或状态机
     *
     * Hardware/IR_Module.c 中现有的红外状态机：
     *   - 读取 4 位传感器状态
     *   - 分类：十字、90度左转、90度右转、大左、大右、小左、小右、直行、丢失
     *   - 设置 turn_diff 和 base_speed_mm
     *   - 计算左右目标速度 (m/s)
     *   - 写入 MotorA.Target_Encoder, MotorB.Target_Encoder
     *
     * 对于新架构，本函数应：
     *   1. 将 sensor_state 分类为线位置误差
     *   2. 使用 PID 计算转弯修正量
     *   3. 返回修正值 或 直接调用 MotionControl_SetTarget()
     *
     * 目前先委托给现有的 IRDM_line_inspection() 逻辑。
     */

    /* STUB: 调用现有的红外模块逻辑 */
    IR_LineDetect_Update();

    return turn_diff;
}

void LineFollow_SetBaseSpeed(float speed_mps)
{
    if (speed_mps > 0.0f && speed_mps <= MAX_LINEAR_SPEED_MPS) {
        g_base_speed = speed_mps;
    }
}

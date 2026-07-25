/**
 * @file    line_follow.c
 * @brief   循线控制策略
 *
 * 负责人：团队成员
 * 状态：已完成 — 从 Hardware/IR_Module.c 迁移
 *
 * 本模块是应用层的桥梁，连接：
 *   Hardware/ir_track.c        （传感器读取 + 分类）
 *   Application/motion_control.c（PID 速度控制）
 *
 * 流程（从状态机 Handle_LineFollow 调用，200Hz）：
 *   1. IR_LineDetect_Update()   — 读取传感器、分类、计算底盘指令
 *   2. IR_GetLineFollowCmd()    — 获取计算出的指令
 *   3.（可选）对位置误差进行 PID 修正
 *   4. MotionControl_SetTarget() — 送入速度 PID 环
 *
 * 红外循线状态机（来自 IR_Module.c 的 IRDM_line_inspection）
 * 已移至 Hardware/ir_track.c 以保持正确的分层结构。
 */

#include "line_follow.h"
#include "ir_track.h"
#include "motion_control.h"
#include "kinematics.h"
#include "robot_config.h"
#include "pid_config.h"
#include "pid.h"
#include <math.h>
#include <stdbool.h>

/* ========================================================================
 * 模块变量
 * ======================================================================== */
static float g_base_speed = 0.15f;    /* 基础巡航速度 (m/s) */
static PID_t g_line_pid;              /* 位置修正 PID       */
static bool   g_initialized = false;

/* 外部 turn_diff（来自 IR_Module.c，同时在 ir_track.c 中计算） */
extern float turn_diff;
extern int   Flag_Stop;

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void LineFollow_Init(void)
{
    g_base_speed = 0.15f;  /* 默认 150mm/s */

    /* 初始化一个用于线位置修正的 PID。
     * 输入：位置误差 (mm)
     * 输出：转弯速率修正 (rad/s) */
    PID_Init(&g_line_pid,
             LINE_KP_DEFAULT, LINE_KI_DEFAULT, LINE_KD_DEFAULT,
             LINE_OUT_MIN, LINE_OUT_MAX);

    g_initialized = true;
}

/**
 * @brief  计算循线修正量并设置运动目标
 * @param  sensor_state  4 位红外传感器值 (0x0 ~ 0xF)
 * @return 速度修正值（m/s，正值 = 右转）
 *
 * 这是主要的应用层循线函数。
 *
 * 策略：
 *   1. 调用 IR_LineDetect_Update() — 硬件层分类
 *      （读取传感器、识别转弯类型、计算底盘指令）
 *   2. 通过 IR_GetLineFollowCmd() 获取底盘指令
 *   3. 应用额外位置误差 PID 修正（可选，用于平滑跟踪）
 *   4. 将最终指令送入 MotionControl_SetTarget()
 */
float LineFollow_ComputeCorrection(uint8_t sensor_state)
{
    if (!g_initialized) {
        LineFollow_Init();
    }

    /* ---- 第 1 步：运行硬件层传感器分类 ----
     * 读取 4 路红外 GPIO，分类线状态，
     * 计算 base_speed 和 turn_diff，并存储 ChassisCmd_t。 */
    IR_LineDetect_Update();

    /* ---- 第 2 步：获取计算出的底盘指令 ---- */
    ChassisCmd_t cmd = IR_GetLineFollowCmd();

    /* ---- 第 3 步（可选）：使用位置误差 PID 进行微调 ----
     * 来自 IR_LineDetect_Update 的原始 cmd 使用启发式转弯模型。
     * 我们可以选择在此基础上叠加 PID 以获得更平滑的循线效果。
     *
     * float pos_error = IR_GetPositionError();       // mm
     * float pos_correction = PID_Compute(&g_line_pid, pos_error, CONTROL_DT_S);
     * cmd.wz += pos_correction * 0.01f;              // 缩放 mm→rad/s
     *
     * 目前启发式模型效果已足够好。如果机器人出现振荡
     * 或转向过度/不足，再启用 PID 修正。 */

    /* 限幅角速度 */
    float max_wz = MAX_ANGULAR_SPEED_RPS;
    if (cmd.wz >  max_wz) cmd.wz =  max_wz;
    if (cmd.wz < -max_wz) cmd.wz = -max_wz;

    /* ---- 第 4 步：送入运动控制器 ---- */
    MotionControl_SetTarget(cmd);

    /* 返回转弯差速用于显示/调试 */
    return IR_GetTurnDiff();
}

void LineFollow_SetBaseSpeed(float speed_mps)
{
    if (speed_mps > 0.0f && speed_mps <= MAX_LINEAR_SPEED_MPS) {
        g_base_speed = speed_mps;
        /* 同时更新硬件层基础速度（单位 mm/s） */
        IR_SetBaseSpeed(speed_mps * 1000.0f);
    }
}

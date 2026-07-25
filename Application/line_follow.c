/**
 * @file    line_follow.c
 * @brief   Line-following control strategy
 *
 * OWNER:  Team Member
 * STATUS: COMPLETE — migrated from Hardware/IR_Module.c
 *
 * This module is the Application-layer bridge between:
 *   Hardware/ir_track.c   (sensor reading + classification)
 *   Application/motion_control.c (PID velocity control)
 *
 * Flow (called from state machine Handle_LineFollow, 200Hz):
 *   1. IR_LineDetect_Update()   — read sensors, classify, compute chassis cmd
 *   2. IR_GetLineFollowCmd()    — retrieve computed cmd
 *   3. (optional) PID correction on position error
 *   4. MotionControl_SetTarget() — feed to velocity PID loop
 *
 * The IR line-follow state machine (from IR_Module.c IRDM_line_inspection)
 * has been moved into Hardware/ir_track.c for proper layering.
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
 * Module Variables
 * ======================================================================== */
static float g_base_speed = 0.15f;    /* Base cruising speed (m/s) */
static PID_t g_line_pid;              /* Position-correction PID   */
static bool   g_initialized = false;

/* Extern turn_diff (from IR_Module.c, also computed in ir_track.c) */
extern float turn_diff;
extern int   Flag_Stop;

/* ========================================================================
 * 公开函数
 * Public Functions
 * ======================================================================== */

void LineFollow_Init(void)
{
    g_base_speed = 0.15f;  /* Default 150mm/s */

    /* Initialize a PID for line-position correction.
     * Input:  position error (mm)
     * Output: turn rate correction (rad/s) */
    PID_Init(&g_line_pid,
             LINE_KP_DEFAULT, LINE_KI_DEFAULT, LINE_KD_DEFAULT,
             LINE_OUT_MIN, LINE_OUT_MAX);

    g_initialized = true;
}

/**
 * @brief  Compute line-follow correction and set motion target
 * @param  sensor_state  4-bit IR sensor value (0x0 ~ 0xF)
 * @return Speed correction value (m/s, positive = turn right)
 *
 * This is the main Application-layer line-follow function.
 *
 * Strategy:
 *   1. Call IR_LineDetect_Update() — Hardware-layer classification
 *      (reads sensors, identifies turn type, computes chassis command)
 *   2. Retrieve chassis command via IR_GetLineFollowCmd()
 *   3. Apply additional position-error PID correction (optional, for smoothness)
 *   4. Feed final command to MotionControl_SetTarget()
 */
float LineFollow_ComputeCorrection(uint8_t sensor_state)
{
    if (!g_initialized) {
        LineFollow_Init();
    }

    /* ---- Step 1: Run Hardware-layer sensor classification ----
     * This reads the 4 IR GPIOs, classifies the line state,
     * computes base_speed and turn_diff, and stores a ChassisCmd_t. */
    IR_LineDetect_Update();

    /* ---- Step 2: Retrieve the computed chassis command ---- */
    ChassisCmd_t cmd = IR_GetLineFollowCmd();

    /* ---- Step 3 (optional): Fine-tune with position-error PID ----
     * The raw cmd from IR_LineDetect_Update uses a heuristic turn model.
     * We can optionally layer a PID on top for smoother line tracking.
     *
     * float pos_error = IR_GetPositionError();       // mm
     * float pos_correction = PID_Compute(&g_line_pid, pos_error, CONTROL_DT_S);
     * cmd.wz += pos_correction * 0.01f;              // scale mm→rad/s
     *
     * For now, the heuristic model works well enough. Enable PID
     * correction if the robot oscillates or over/under-steers. */

    /* Clamp angular velocity */
    float max_wz = MAX_ANGULAR_SPEED_RPS;
    if (cmd.wz >  max_wz) cmd.wz =  max_wz;
    if (cmd.wz < -max_wz) cmd.wz = -max_wz;

    /* ---- Step 4: Feed to motion controller ---- */
    MotionControl_SetTarget(cmd);

    /* Return the turn differential for display/debug */
    return IR_GetTurnDiff();
}

void LineFollow_SetBaseSpeed(float speed_mps)
{
    if (speed_mps > 0.0f && speed_mps <= MAX_LINEAR_SPEED_MPS) {
        g_base_speed = speed_mps;
        /* Also update Hardware-layer base speed (in mm/s) */
        IR_SetBaseSpeed(speed_mps * 1000.0f);
    }
}

/**
 * @file    line_follow.c
 * @brief   Line-following strategy — STUB for team member
 *
 * OWNER:  Team Member
 * STATUS: STUB — fill in with real line-following algorithm
 *
 * TODO: The existing IR line-follow state machine is in Hardware/IR_Module.c
 *       (function IRDM_line_inspection()). Migrate and enhance it here.
 */

#include "line_follow.h"
#include "ir_track.h"
#include "motion_control.h"
#include "kinematics.h"
#include "robot_config.h"

/* ========================================================================
 * Module Variables
 * ======================================================================== */
static float g_base_speed = 0.15f;  /* m/s, default 150mm/s */

extern float turn_diff;
extern int Flag_Stop;

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void LineFollow_Init(void)
{
    g_base_speed = 0.15f;
}

float LineFollow_ComputeCorrection(uint8_t sensor_state)
{
    /* TODO: Implement line-following PID or state machine
     *
     * The existing IR state machine in Hardware/IR_Module.c:
     *   - Reads 4-bit sensor state
     *   - Classifies: cross, 90-left, 90-right, big-left, big-right,
     *                 small-left, small-right, straight, lost
     *   - Sets turn_diff and base_speed_mm
     *   - Computes left/right target speeds (m/s)
     *   - Writes to MotorA.Target_Encoder, MotorB.Target_Encoder
     *
     * For the new architecture, this function should:
     *   1. Classify sensor_state into a line position error
     *   2. Use a PID to compute turn correction
     *   3. Return correction value OR directly call MotionControl_SetTarget()
     *
     * For now, delegate to the existing IRDM_line_inspection() logic.
     */

    /* STUB: Call existing IR module logic */
    IR_LineDetect_Update();

    return turn_diff;
}

void LineFollow_SetBaseSpeed(float speed_mps)
{
    if (speed_mps > 0.0f && speed_mps <= MAX_LINEAR_SPEED_MPS) {
        g_base_speed = speed_mps;
    }
}

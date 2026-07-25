/**
 * @file    ir_track.c
 * @brief   Infrared line tracking sensor driver (4-channel)
 *
 * Implements the API declared in ir_track.h:
 *   - Raw sensor reading (GPIO)
 *   - Sensor state classification (cross, left/right turns, straight, lost)
 *   - Line-following state machine (migrated from IR_Module.c IRDM_line_inspection)
 *
 * Hardware: 4 digital IR sensors (DH1 ~ DH4)
 *   - DH1: PA27  (bit3)
 *   - DH2: PA12  (bit2)
 *   - DH3: PB16  (bit1)
 *   - DH4: PB17  (bit0)
 *
 * Layer: Hardware — only calls GPIO drivers + Middleware (kinematics)
 */

#include "ir_track.h"
#include "kinematics.h"
#include "robot_config.h"
#include "ti_msp_dl_config.h"
#include <math.h>

/* ========================================================================
 * Module Variables — sensor state (for display/external access)
 * ======================================================================== */
uint32_t ir_dh1_state;
uint32_t ir_dh2_state;
uint32_t ir_dh3_state;
uint32_t ir_dh4_state;

/* ========================================================================
 * Line-Follow State Machine Variables
 * ======================================================================== */
static float g_base_speed = 0.15f;       /* Base speed in mm/s (for compat), actually m/s */
static float g_turn_diff  = 0.0f;        /* Current turn differential */
static int   g_last_state = 0;           /* Previous sensor state for lost-line recovery */
static int   g_turn_cnt   = 0;           /* Turn counter for 90-degree handling */
static int   g_saved_state = 0;          /* Saved turn state during countdown */

/* ---- Tunable parameters — defined in IR_Module.c during migration
 *     (will move here when legacy code is removed) ---- */
extern float Turn90Angle;
extern float TurnMaxAngle;
extern float TurnMidAngle;
extern float TurnMinAngle;
extern float BaseSpeed;      /* mm/s */
extern float ForwardLimit;

/* ---- Output: target chassis command (read by Application/motion_control) ---- */
static ChassisCmd_t g_linefollow_cmd;

/* ========================================================================
 * Public Functions — Raw Sensor
 * ======================================================================== */

/**
 * @brief  Read 4 IR sensor GPIOs and pack into a 4-bit state
 * @return 4-bit: bit3=DH1, bit2=DH2, bit1=DH3, bit0=DH4
 *         1 = black line detected, 0 = white ground
 */
uint8_t IR_GetSensorState(void)
{
    ir_dh1_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0;
    ir_dh2_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0;
    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0;
    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0;

    return (uint8_t)((ir_dh1_state << 3) | (ir_dh2_state << 2)
                   | (ir_dh3_state << 1) |  ir_dh4_state);
}

/**
 * @brief  Convert sensor state to position error in mm
 * @return Position error: 0=centered, negative=left deviation, positive=right
 *
 * Weighted sum of sensor positions. DH1/DH4 are the outer sensors,
 * DH2/DH3 are inner. Each sensor has a weight proportional to its
 * distance from the centerline.
 */
float IR_GetPositionError(void)
{
    /* Read current sensor state */
    uint8_t state = IR_GetSensorState();

    /* Weight lookup: assume sensors spaced evenly across sensor bar.
     * DH1=far_left(-30mm), DH2=near_left(-10mm), DH3=near_right(+10mm), DH4=far_right(+30mm) */
    float error = 0.0f;
    int   count = 0;

    if (state & 0x08) { error -= 30.0f; count++; }  /* DH1: far left  */
    if (state & 0x04) { error -= 10.0f; count++; }  /* DH2: near left */
    if (state & 0x02) { error += 10.0f; count++; }  /* DH3: near right*/
    if (state & 0x01) { error += 30.0f; count++; }  /* DH4: far right */

    /* Average over active sensors */
    if (count > 0) {
        error /= (float)count;
    }

    return error;
}

/* ========================================================================
 * Public Functions — Line-Follow State Machine
 * ======================================================================== */

/**
 * @brief  Set base cruising speed for line following
 * @param  speed_mmps  Speed in mm/s (internally converted to m/s)
 */
void IR_SetBaseSpeed(float speed_mmps)
{
    if (speed_mmps > 0.0f && speed_mmps <= (MAX_LINEAR_SPEED_MPS * 1000.0f)) {
        g_base_speed = speed_mmps / 1000.0f;  /* mm/s → m/s */
        BaseSpeed    = speed_mmps;             /* Keep legacy variable in sync */
    }
}

/**
 * @brief  Get current turn differential
 * @return Turn differential (degree-equivalent, positive = turn left)
 */
float IR_GetTurnDiff(void)
{
    return g_turn_diff;
}

/**
 * @brief  Run one iteration of the line-following state machine
 * @note   Call at CONTROL_FREQ_HZ (200Hz) from timer ISR
 *
 * Migrated from IR_Module.c IRDM_line_inspection().
 * Classifies 4-bit sensor state and computes chassis motion command.
 *
 * The resulting ChassisCmd_t is stored globally for
 * Application/motion_control.c to pick up via LineFollow_ComputeCorrection().
 *
 * Classification:
 *   0x00 = Cross (all white)
 *   0x01 = Left 90° A      0x03 = Left 90° B
 *   0x08 = Right 90° A     0x0C = Right 90° B
 *   0x07 = Left Big        0x0E = Right Big
 *   0x0B = Left Small      0x0D = Right Small
 *   0x09 = Straight        0x0F = Lost (all black)
 */
void IR_LineDetect_Update(void)
{
    uint8_t sensor_state = IR_GetSensorState();
    float   base_speed_mm;
    float   left_motor_speed;
    float   right_motor_speed;

    /* ---- 90-degree turn handling (straight-200-then-turn) ---- */
    if ((sensor_state == 0x01 || sensor_state == 0x08
      || sensor_state == 0x03 || sensor_state == 0x0C) && g_turn_cnt == 0)
    {
        g_saved_state = sensor_state;
        g_turn_cnt = 1;
    }

    if (g_turn_cnt > 0)
    {
        if (g_turn_cnt < 175) {
            sensor_state = 0x09;  /* Force straight for ~200 cycles */
        } else if (g_turn_cnt < 4000
                && sensor_state != 0x07 && sensor_state != 0x0E) {
            sensor_state = g_saved_state;
        } else {
            g_turn_cnt = 0;
            g_saved_state = 0;
        }
        if (g_turn_cnt > 0) g_turn_cnt++;
    }

    /* ---- Classify sensor state → turn differential ---- */
    switch (sensor_state)
    {
        case 0x00:  /* Cross / all white */
            g_turn_diff = 0.0f;
            break;

        case 0x01:  /* Left 90° A */
        case 0x03:  /* Left 90° B */
            g_turn_diff = Turn90Angle;
            break;

        case 0x08:  /* Right 90° A */
        case 0x0C:  /* Right 90° B */
            g_turn_diff = -Turn90Angle;
            break;

        case 0x07:  /* Left big turn */
            g_turn_diff = TurnMaxAngle;
            break;

        case 0x0E:  /* Right big turn */
            g_turn_diff = -TurnMaxAngle;
            break;

        case 0x0B:  /* Left small turn */
            g_turn_diff = TurnMinAngle;
            break;

        case 0x0D:  /* Right small turn */
            g_turn_diff = -TurnMinAngle;
            break;

        case 0x09:  /* Straight */
            g_turn_diff = 0.0f;
            break;

        case 0x0F:  /* Lost (all black) — use last known direction */
            if (g_last_state == 0x0B)           g_turn_diff =  TurnMidAngle;
            else if (g_last_state == 0x0D)      g_turn_diff = -TurnMidAngle;
            else if (g_last_state == 0x07)      g_turn_diff =  TurnMaxAngle;
            else if (g_last_state == 0x0E)      g_turn_diff = -TurnMaxAngle;
            break;

        default:  /* Unrecognized — go straight */
            g_turn_diff = 0.0f;
            break;
    }

    /* Remember state for lost-line recovery */
    if (sensor_state != 0x0F) {
        g_last_state = sensor_state;
    }

    /* ---- Compute base speed: sharper turn → slower speed ---- */
    if (fabsf(g_turn_diff) < ForwardLimit) {
        base_speed_mm = BaseSpeed * (1.0f - fabsf(g_turn_diff) / ForwardLimit);
        if (base_speed_mm < 0.0f) base_speed_mm = 0.0f;
    } else {
        base_speed_mm = 0.0f;
    }

    /* ---- Differential drive: left = base - turn, right = base + turn ----
     * Turn convention: positive turn_diff = left turn
     *   → left motor slower, right motor faster
     * Speed is in mm/s input, convert to m/s for kinematics API */
    left_motor_speed  = 0.001f * (base_speed_mm - g_turn_diff);
    right_motor_speed = 0.001f * (base_speed_mm + g_turn_diff);

    /* ---- Clamp individual wheel speeds ---- */
    if (left_motor_speed  >  MAX_LINEAR_SPEED_MPS) left_motor_speed  =  MAX_LINEAR_SPEED_MPS;
    if (left_motor_speed  < -MAX_LINEAR_SPEED_MPS) left_motor_speed  = -MAX_LINEAR_SPEED_MPS;
    if (right_motor_speed >  MAX_LINEAR_SPEED_MPS) right_motor_speed =  MAX_LINEAR_SPEED_MPS;
    if (right_motor_speed < -MAX_LINEAR_SPEED_MPS) right_motor_speed = -MAX_LINEAR_SPEED_MPS;

    /* ---- Compute chassis command via forward kinematics ----
     * vx = (V_left + V_right) / 2
     * wz = (V_right - V_left) / WheelSpacing */
    WheelSpeed_t wheels;
    wheels.left  = left_motor_speed;
    wheels.right = right_motor_speed;

    g_linefollow_cmd = Kinematics_Forward(wheels);
}

/* ========================================================================
 * Accessor for Application layer (line_follow.c / state_machine.c)
 * ======================================================================== */

/**
 * @brief  Get the last computed line-follow chassis command
 * @return ChassisCmd_t with (vx, wz) ready for motion control
 *
 * Call this after IR_LineDetect_Update() to feed MotionControl_SetTarget().
 */
ChassisCmd_t IR_GetLineFollowCmd(void)
{
    return g_linefollow_cmd;
}

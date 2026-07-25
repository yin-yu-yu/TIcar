/**
 * @file    state_machine.c
 * @brief   Robot state machine implementation
 *
 * OWNER:  Team Lead (队长)
 * STATUS: Active development — fill in transition logic per competition rules
 *
 * This is the BRAIN of the robot. Each state handler calls into
 * lower-layer modules (MotionControl, LineFollow, BT_Protocol)
 * which team members fill in.
 */

#include "state_machine.h"
#include "robot_config.h"
#include "pid_config.h"

/* ---- Hardware ---- */
#include "motor.h"
#include "led.h"
#include "oled.h"

/* ---- Middleware ---- */
#include "kinematics.h"

/* ---- Application ---- */
#include "motion_control.h"
#include "line_follow.h"
#include "bt_protocol.h"

/* ---- Key flag externs (from existing key.c / uart_callback.c) ---- */
extern uint8_t Flag_Stop;
extern int    Flag_Direction, Flag_Left, Flag_Right, Turn_Flag;
extern int    Run_Mode;
extern float  RC_Velocity, Move_X, Move_Z;

/* ========================================================================
 * Module Variables
 * ======================================================================== */

static struct {
    RobotState_t current;
    RobotState_t previous;
    uint32_t     entry_tick;       /* SysTick count when entering state     */
    float        battery_voltage;  /* Latest battery reading (V)            */
    bool         low_battery;      /* Below BATT_WARN_THRESHOLD_V flag      */
} g_sm;

/* ========================================================================
 * Forward Declarations (State Handlers)
 * ======================================================================== */
static void Handle_Init(void);
static void Handle_Idle(void);
static void Handle_RC_Drive(void);
static void Handle_LineFollow(void);
static void Handle_LowBattery(void);
static void Handle_Error(void);

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void SM_Init(void)
{
    g_sm.current       = STATE_INIT;
    g_sm.previous      = STATE_INIT;
    g_sm.entry_tick    = 0;
    g_sm.battery_voltage = 8.4f;  /* Assume full battery at startup */
    g_sm.low_battery   = false;

    Kinematics_Init();
    Odom_Init();

    /* TODO: Call team member module inits when ready
     * MPU6050_Init();
     * BT_Protocol_Init();
     * MotionControl_Init();
     * LineFollow_Init(); */
}

void SM_Run(void)
{
    RobotState_t state = g_sm.current;

    /* ---- Evaluate low battery condition (from any state) ---- */
    if (state != STATE_INIT && state != STATE_ERROR) {
        if (g_sm.battery_voltage < BATT_CRITICAL_THRESHOLD_V) {
            SM_Transition(STATE_ERROR);
            state = STATE_ERROR;
        } else if (g_sm.battery_voltage < BATT_WARN_THRESHOLD_V
                   && state != STATE_LOW_BATTERY) {
            SM_Transition(STATE_LOW_BATTERY);
            state = STATE_LOW_BATTERY;
        }
    }

    /* ---- Dispatch to current state handler ---- */
    switch (state) {
        case STATE_INIT:          Handle_Init();         break;
        case STATE_IDLE:          Handle_Idle();         break;
        case STATE_RC_DRIVE:      Handle_RC_Drive();     break;
        case STATE_LINE_FOLLOW:   Handle_LineFollow();   break;
        case STATE_LOW_BATTERY:   Handle_LowBattery();   break;
        case STATE_ERROR:         Handle_Error();        break;
        default:                                            break;
    }
}

void SM_Transition(RobotState_t next)
{
    if (next >= STATE_COUNT) return;
    if (next == g_sm.current) return;

    /* On exit from current state */
    switch (g_sm.current) {
        case STATE_RC_DRIVE:
        case STATE_LINE_FOLLOW:
            Motor_Stop();  /* Always stop motors when leaving drive state */
            break;
        default:
            break;
    }

    g_sm.previous  = g_sm.current;
    g_sm.current   = next;
    g_sm.entry_tick = 0;  /* TODO: use actual tick if needed for timing */

    /* On entry to new state */
    switch (next) {
        case STATE_IDLE:
            Motor_Stop();
            LED_OFF();
            break;
        case STATE_LOW_BATTERY:
            Motor_Stop();
            break;
        case STATE_ERROR:
            Motor_Stop();
            break;
        default:
            break;
    }
}

RobotState_t SM_GetState(void)
{
    return g_sm.current;
}

const char* SM_GetStateName(RobotState_t state)
{
    switch (state) {
        case STATE_INIT:          return "INIT";
        case STATE_IDLE:          return "IDLE";
        case STATE_RC_DRIVE:      return "RC_DRIVE";
        case STATE_LINE_FOLLOW:   return "LINE_FW";
        case STATE_LOW_BATTERY:   return "LOW_BAT";
        case STATE_ERROR:         return "ERROR";
        default:                  return "????";
    }
}

bool SM_IsMoving(void)
{
    return (g_sm.current == STATE_RC_DRIVE
         || g_sm.current == STATE_LINE_FOLLOW);
}

void SM_ReportBattery(float voltage)
{
    g_sm.battery_voltage = voltage;
}

/* ========================================================================
 * State Handlers (private)
 * ======================================================================== */

/**
 * STATE_INIT — Power-on self-test
 * Transition: always → STATE_IDLE (for now)
 * TODO: Add sensor checks before transitioning
 */
static void Handle_Init(void)
{
    /* Quick self-check: verify battery present */
    if (g_sm.battery_voltage > BATT_CRITICAL_THRESHOLD_V) {
        SM_Transition(STATE_IDLE);
    } else {
        /* Stay in INIT, battery too low even to idle */
    }
}

/**
 * STATE_IDLE — Standby
 * - Wait for BT command or key press to change mode
 * - Key: single-click = stop/start toggle, double-click = mode switch
 * - BT: APP command can trigger RC_DRIVE
 * - Transition: → STATE_RC_DRIVE / STATE_LINE_FOLLOW
 */
static void Handle_Idle(void)
{
    /* Check BT flag for RC mode activation
     * (Flag_Direction != 0 means APP joystick is active) */
    if (Run_Mode == 0 && Flag_Direction != 0 && !Flag_Stop) {
        SM_Transition(STATE_RC_DRIVE);
    }
    /* Check if IR line-follow mode selected */
    else if (Run_Mode == 1 && !Flag_Stop) {
        SM_Transition(STATE_LINE_FOLLOW);
    }
}

/**
 * STATE_RC_DRIVE — Remote control via Bluetooth APP
 * - Reads APP joystick commands (Flag_Direction, Flag_Left, Flag_Right)
 * - Translates to chassis motion command
 * - Sends to MotionControl
 * - Transition: → STATE_IDLE (on stop or mode change)
 */
static void Handle_RC_Drive(void)
{
    ChassisCmd_t cmd;
    float vel = RC_Velocity / 1000.0f;  /* mm/s → m/s */

    /* If stop flag set or mode changed, go back to idle */
    if (Flag_Stop || Run_Mode != 0) {
        SM_Transition(STATE_IDLE);
        return;
    }

    /* ---- Translate APP direction flags to chassis command ----
     * Flag_Direction: 1=forward, 2=forward-right, 3=right, ... 8=forward-left
     * This is a simplified mapping; team member refines in bt_protocol.c */
    switch (Flag_Direction) {
        case 1:  cmd.vx =  vel;  cmd.wz =  0.0f;        break;  /* Forward  */
        case 2:  cmd.vx =  vel;  cmd.wz = -1.0f;         break;  /* Fwd-Right*/
        case 3:  cmd.vx =  0.0f; cmd.wz = -1.0f;         break;  /* Right    */
        case 4:  cmd.vx = -vel;  cmd.wz = -1.0f;         break;  /* Bwd-Right*/
        case 5:  cmd.vx = -vel;  cmd.wz =  0.0f;         break;  /* Backward */
        case 6:  cmd.vx = -vel;  cmd.wz =  1.0f;         break;  /* Bwd-Left */
        case 7:  cmd.vx =  0.0f; cmd.wz =  1.0f;         break;  /* Left     */
        case 8:  cmd.vx =  vel;  cmd.wz =  1.0f;         break;  /* Fwd-Left */
        default: cmd.vx =  0.0f; cmd.wz =  0.0f;         break;
    }

    /* Additional turn flags from APP steering mode */
    if (Flag_Left)   cmd.wz =  1.5f;
    if (Flag_Right)  cmd.wz = -1.5f;

    MotionControl_SetTarget(cmd);
}

/**
 * STATE_LINE_FOLLOW — Autonomous line following
 * - Reads IR sensor via LineFollow module
 * - Computes speed correction
 * - Transition: → STATE_IDLE (on stop or mode change)
 */
static void Handle_LineFollow(void)
{
    if (Flag_Stop || Run_Mode != 1) {
        SM_Transition(STATE_IDLE);
        return;
    }

    /* LineFollow_ComputeCorrection() is filled by team member
     * It internally calls MotionControl_SetTarget() */
    IR_LineDetect_Update();
}

/**
 * STATE_LOW_BATTERY — Low battery warning
 * - LED flashes, OLED shows warning
 * - Motors STOPPED (safety)
 * - Can still transition to IDLE or RC_DRIVE if user insists
 * - Transition: → STATE_IDLE (when battery recovers above threshold)
 */
static void Handle_LowBattery(void)
{
    LED_Flash(500);  /* Flash LED at 2Hz */

    /* Check if battery recovered */
    if (g_sm.battery_voltage >= BATT_WARN_THRESHOLD_V + 0.2f) {
        LED_OFF();
        SM_Transition(STATE_IDLE);
    }
}

/**
 * STATE_ERROR — Critical fault
 * - Motors STOPPED
 * - LED solid ON
 * - Can only exit via reset or battery recovery
 */
static void Handle_Error(void)
{
    Motor_Stop();
    LED_ON();

    /* Recovery: battery back above warning */
    if (g_sm.battery_voltage >= BATT_WARN_THRESHOLD_V) {
        LED_OFF();
        SM_Transition(STATE_IDLE);
    }
}

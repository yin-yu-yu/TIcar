/**
 * @file    state_machine.c
 * @brief   机器人状态机实现
 *
 * 负责人：队长
 * 状态：积极开发中 — 按比赛规则填充状态转换逻辑
 *
 * 这是机器人的"大脑"。每个状态处理函数调用底层模块
 *（MotionControl, LineFollow, BT_Protocol），
 * 由团队成员负责填充。
 */

#include "state_machine.h"
#include "robot_config.h"
#include "pid_config.h"

/* ---- 硬件层 ---- */
#include "motor.h"
#include "led.h"
#include "oled.h"

/* ---- 中间件 ---- */
#include "kinematics.h"

/* ---- 应用层 ---- */
#include "motion_control.h"
#include "line_follow.h"
#include "bt_protocol.h"

/* ---- 关键标志位 extern（来自现有 key.c / uart_callback.c）---- */
extern uint8_t Flag_Stop;
extern int    Flag_Direction, Flag_Left, Flag_Right, Turn_Flag;
extern int    Run_Mode;
extern float  RC_Velocity, Move_X, Move_Z;

/* ========================================================================
 * 模块变量
 * ======================================================================== */

static struct {
    RobotState_t current;
    RobotState_t previous;
    uint32_t     entry_tick;       /* 进入状态时的 SysTick 计数     */
    float        battery_voltage;  /* 最新电池电压读数 (V)           */
    bool         low_battery;      /* 低于 BATT_WARN_THRESHOLD_V 标志 */
} g_sm;

/* ========================================================================
 * 前向声明（状态处理函数）
 * ======================================================================== */
static void Handle_Init(void);
static void Handle_Idle(void);
static void Handle_RC_Drive(void);
static void Handle_LineFollow(void);
static void Handle_LowBattery(void);
static void Handle_Error(void);

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void SM_Init(void)
{
    g_sm.current       = STATE_INIT;
    g_sm.previous      = STATE_INIT;
    g_sm.entry_tick    = 0;
    g_sm.battery_voltage = 8.4f;  /* 启动时假设电池满电 */
    g_sm.low_battery   = false;

    Kinematics_Init();
    Odom_Init();

    /* ---- 初始化子模块（团队成员）---- */
    MotionControl_Init();
    LineFollow_Init();
    BT_Protocol_Init();
    MPU6050_Init();   /* IMU: I2C 初始化 + 寄存器配置 (内部标志防重复调用) */
}

void SM_Run(void)
{
    RobotState_t state = g_sm.current;

    /* ---- 评估低电量条件（所有状态通用）---- */
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

    /* ---- 分发到当前状态处理函数 ---- */
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

    /* 离开当前状态时 */
    switch (g_sm.current) {
        case STATE_RC_DRIVE:
        case STATE_LINE_FOLLOW:
            Motor_Stop();  /* 离开驱动状态时总是停止电机 */
            break;
        default:
            break;
    }

    g_sm.previous  = g_sm.current;
    g_sm.current   = next;
    g_sm.entry_tick = 0;  /* TODO: 如需计时功能，使用实际的 tick 值 */

    /* 进入新状态时 */
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
 * 状态处理函数（私有）
 * ======================================================================== */

/**
 * STATE_INIT — 上电自检
 * 转换：始终 → STATE_IDLE（暂定）
 * TODO: 转换前添加传感器检查
 */
static void Handle_Init(void)
{
    /* 快速自检：验证电池是否接入 */
    if (g_sm.battery_voltage > BATT_CRITICAL_THRESHOLD_V) {
        SM_Transition(STATE_IDLE);
    } else {
        /* 保持在 INIT，电池电量过低，甚至不能进入空闲 */
    }
}

/**
 * STATE_IDLE — 待机
 * - 等待蓝牙命令或按键切换模式
 * - 按键：单击 = 启停切换，双击 = 模式切换
 * - 蓝牙：APP 命令可触发 RC_DRIVE
 * - 转换：→ STATE_RC_DRIVE / STATE_LINE_FOLLOW
 */
static void Handle_Idle(void)
{
    /* 检查蓝牙标志以激活遥控模式
     *（Flag_Direction != 0 表示 APP 摇杆处于活动状态） */
    if (Run_Mode == 0 && Flag_Direction != 0 && !Flag_Stop) {
        SM_Transition(STATE_RC_DRIVE);
    }
    /* 检查是否选择了红外循线模式 */
    else if (Run_Mode == 1 && !Flag_Stop) {
        SM_Transition(STATE_LINE_FOLLOW);
    }
}

/**
 * STATE_RC_DRIVE — 通过蓝牙 APP 遥控
 * - 读取 APP 摇杆命令（Flag_Direction, Flag_Left, Flag_Right）
 * - 转换为底盘运动指令
 * - 发送给 MotionControl
 * - 转换：→ STATE_IDLE（停止或模式改变时）
 */
static void Handle_RC_Drive(void)
{
    ChassisCmd_t cmd;
    float vel = RC_Velocity / 1000.0f;  /* mm/s → m/s */

    /* 如果停止标志置位或模式改变，返回空闲 */
    if (Flag_Stop || Run_Mode != 0) {
        SM_Transition(STATE_IDLE);
        return;
    }

    /* ---- 将 APP 方向标志转换为底盘指令 ----
     * Flag_Direction: 1=前进, 2=右前, 3=右, ... 8=左前
     * 这是简化映射；团队成员在 bt_protocol.c 中细化 */
    switch (Flag_Direction) {
        case 1:  cmd.vx =  vel;  cmd.wz =  0.0f;        break;  /* 前进 */
        case 2:  cmd.vx =  vel;  cmd.wz = -1.0f;         break;  /* 右前 */
        case 3:  cmd.vx =  0.0f; cmd.wz = -1.0f;         break;  /* 右   */
        case 4:  cmd.vx = -vel;  cmd.wz = -1.0f;         break;  /* 右后 */
        case 5:  cmd.vx = -vel;  cmd.wz =  0.0f;         break;  /* 后退 */
        case 6:  cmd.vx = -vel;  cmd.wz =  1.0f;         break;  /* 左后 */
        case 7:  cmd.vx =  0.0f; cmd.wz =  1.0f;         break;  /* 左   */
        case 8:  cmd.vx =  vel;  cmd.wz =  1.0f;         break;  /* 左前 */
        default: cmd.vx =  0.0f; cmd.wz =  0.0f;         break;
    }

    /* APP 转向模式的额外转弯标志 */
    if (Flag_Left)   cmd.wz =  1.5f;
    if (Flag_Right)  cmd.wz = -1.5f;

    MotionControl_SetTarget(cmd);
}

/**
 * STATE_LINE_FOLLOW — 自主循线
 * - 通过 LineFollow 模块读取红外传感器
 * - 计算速度修正量
 * - 转换：→ STATE_IDLE（停止或模式改变时）
 */
static void Handle_LineFollow(void)
{
    if (Flag_Stop || Run_Mode != 1) {
        SM_Transition(STATE_IDLE);
        return;
    }

    /* 通过应用层读取红外传感器并计算运动指令。
     * LineFollow_ComputeCorrection() 内部：
     *   1. 调用 IR_LineDetect_Update() — 传感器分类
     *   2. 通过 IR_GetLineFollowCmd() 获取 ChassisCmd_t
     *   3. 应用位置误差 PID 修正
     *   4. 调用 MotionControl_SetTarget() — 送入速度 PID 环 */
    uint8_t sensor_state = IR_GetSensorState();
    LineFollow_ComputeCorrection(sensor_state);
}

/**
 * STATE_LOW_BATTERY — 低电量警告
 * - LED 闪烁，OLED 显示警告
 * - 电机停止（安全）
 * - 用户坚持时仍可转换到 IDLE 或 RC_DRIVE
 * - 转换：→ STATE_IDLE（电池恢复到阈值以上时）
 */
static void Handle_LowBattery(void)
{
    LED_Flash(500);  /* 以 2Hz 频率闪烁 LED */

    /* 检查电池是否恢复 */
    if (g_sm.battery_voltage >= BATT_WARN_THRESHOLD_V + 0.2f) {
        LED_OFF();
        SM_Transition(STATE_IDLE);
    }
}

/**
 * STATE_ERROR — 严重故障
 * - 电机停止
 * - LED 常亮
 * - 只能通过复位或电池恢复退出
 */
static void Handle_Error(void)
{
    Motor_Stop();
    LED_ON();

    /* 恢复条件：电池恢复到警告阈值以上 */
    if (g_sm.battery_voltage >= BATT_WARN_THRESHOLD_V) {
        LED_OFF();
        SM_Transition(STATE_IDLE);
    }
}

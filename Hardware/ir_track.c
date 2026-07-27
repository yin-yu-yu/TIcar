/**
 * @file    ir_track.c
 * @brief   红外循迹传感器驱动（四通道）
 *
 * 实现 ir_track.h 中声明的接口：
 *   - 原始传感器读取（GPIO）
 *   - 传感器状态分类（路口、左右转、直行、丢线）
 *   - 循迹状态机（从 IR_Module.c 的 IRDM_line_inspection 迁移）
 *
 * 硬件：四个数字红外传感器（DH1 ~ DH4）
 *   - DH1: PA27  (bit3)
 *   - DH2: PA12  (bit2)
 *   - DH3: PB16  (bit1)
 *   - DH4: PB17  (bit0)
 *
 * 层级：硬件层，仅调用 GPIO 驱动和中间件（运动学）。
 */

#include "ir_track.h"
#include "kinematics.h"
#include "robot_config.h"
#include "ti_msp_dl_config.h"
#include <math.h>

/* ========================================================================
 * 模块变量：传感器状态（供显示/外部访问）
 * ======================================================================== */
/* 原始状态由当前生效的 IR_Module.c 统一持有，避免两套实现重复定义。 */
extern uint32_t ir_dh1_state;
extern uint32_t ir_dh2_state;
extern uint32_t ir_dh3_state;
extern uint32_t ir_dh4_state;

/* ========================================================================
 * 循迹状态机变量
 * ======================================================================== */
static float g_base_speed = 0.15f;       /* 基础速度（为兼容保留 mm/s 命名，实际单位为 m/s） */
static float g_turn_diff  = 0.0f;        /* 当前转向差值 */
static int   g_last_state = 0;           /* 用于丢线恢复的上次传感器状态 */
static int   g_turn_cnt   = 0;           /* 90 度转向处理计数器 */
static int   g_saved_state = 0;          /* 倒计时期间保存的转向状态 */

/* ---- Tunable parameters — defined in IR_Module.c during migration
 *     (will move here when legacy code is removed) ---- */
extern float Turn90Angle;
extern float TurnMaxAngle;
extern float TurnMidAngle;
extern float TurnMinAngle;
extern float BaseSpeed;      /* mm/s */
extern float ForwardLimit;
extern uint32_t TurnStraightCycles;
extern uint32_t TurnHoldMaxCycles;

/* ---- 输出：目标底盘命令（由 Application/motion_control 读取） ---- */
static ChassisCmd_t g_linefollow_cmd;

/* ========================================================================
 * 公共函数：原始传感器
 * ======================================================================== */

/**
 * @brief  读取四个红外传感器 GPIO 并组装为 4 位状态值
 * @return 4 位数：bit3=DH1，bit2=DH2，bit1=DH3，bit0=DH4
 *         1 表示检测到黑线，0 表示白色地面
 */
uint8_t IR_GetSensorState(void)
{
    /* 实车接线中第 1、2 路与原工程标号相反，在读取层交换，
     * 让后续状态表仍按 DH1、DH2、DH3、DH4 的逻辑顺序工作。
     * 传感器为低电平检测黑线：黑线=0，白色反射=1。 */
    ir_dh1_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0;
    ir_dh2_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0;
    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0;
    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0;

    return (uint8_t)((ir_dh1_state << 3) | (ir_dh2_state << 2)
                   | (ir_dh3_state << 1) |  ir_dh4_state);
}

/**
 * @brief  将传感器状态转换为毫米单位的位置误差
 * @return 位置误差：0 为居中，负值为左偏，正值为右偏
 *
 * 对传感器位置进行加权求和。DH1/DH4 为外侧传感器，DH2/DH3 为内侧
 * 传感器；每个传感器的权重与其距中心线的距离成正比。
 */
float IR_GetPositionError(void)
{
    /* 读取当前传感器状态 */
    uint8_t state = IR_GetSensorState();

    /* Weight lookup: assume sensors spaced evenly across sensor bar.
     * DH1=far_left(-30mm), DH2=near_left(-10mm), DH3=near_right(+10mm), DH4=far_right(+30mm) */
    float error = 0.0f;
    int   count = 0;

    if (state & 0x08) { error -= 30.0f; count++; }  /* DH1: far left  */
    if (state & 0x04) { error -= 10.0f; count++; }  /* DH2: near left */
    if (state & 0x02) { error += 10.0f; count++; }  /* DH3: near right*/
    if (state & 0x01) { error += 30.0f; count++; }  /* DH4: far right */

    /* 对已激活传感器求平均值 */
    if (count > 0) {
        error /= (float)count;
    }

    return error;
}

/* ========================================================================
 * 公共函数：循迹状态机
 * ======================================================================== */

/**
 * @brief  设置循迹基础巡航速度
 * @param  speed_mmps  速度（mm/s，内部转换为 m/s）
 */
void IR_SetBaseSpeed(float speed_mmps)
{
    if (speed_mmps > 0.0f && speed_mmps <= (MAX_LINEAR_SPEED_MPS * 1000.0f)) {
        g_base_speed = speed_mmps / 1000.0f;  /* mm/s → m/s */
        BaseSpeed    = speed_mmps;             /* Keep legacy variable in sync */
    }
}

/**
 * @brief  获取当前转向差值
 * @return 转向差值（等效角度，正值表示左转）
 */
float IR_GetTurnDiff(void)
{
    return g_turn_diff;
}

/**
 * @brief  执行一次循迹状态机迭代
 * @note   应在定时器中断中以 CONTROL_FREQ_HZ（200Hz）调用
 *
 * 从 IR_Module.c 的 IRDM_line_inspection() 迁移而来。
 * 对四位传感器状态分类并计算底盘运动命令。
 *
 * 计算得到的 ChassisCmd_t 保存在全局变量中，供
 * Application/motion_control.c 通过 LineFollow_ComputeCorrection() 获取。
 *
 * 分类：
 *   0x00 = 路口（全白）
 *   0x01 = 左 90° A       0x03 = 左 90° B
 *   0x08 = 右 90° A       0x0C = 右 90° B
 *   0x07 = 左大转弯        0x0E = 右大转弯
 *   0x0B = 左小转弯        0x0D = 右小转弯
 *   0x09 = 直行            0x0F = 丢线（全黑）
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
        if ((uint32_t)g_turn_cnt < TurnStraightCycles) {
            sensor_state = 0x09;  /* 直角弯前先直行，可在 IR_Module.c 调节 */
        } else if ((uint32_t)g_turn_cnt < TurnHoldMaxCycles
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

    /* 记录状态，供丢线恢复使用 */
    if (sensor_state != 0x0F) {
        g_last_state = sensor_state;
    }

    /* ---- 计算基础速度：转弯越急，速度越低 ---- */
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

    /* ---- 限制各轮速度 ---- */
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
 * 应用层访问器（line_follow.c / state_machine.c）
 * ======================================================================== */

/**
 * @brief  获取最近一次计算的循迹底盘命令
 * @return 可供运动控制使用的 ChassisCmd_t（vx、wz）
 *
 * 在 IR_LineDetect_Update() 后调用此函数，将结果传给 MotionControl_SetTarget()。
 */
ChassisCmd_t IR_GetLineFollowCmd(void)
{
    return g_linefollow_cmd;
}

#include "IR_Module.h"
#include "control.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;

/* ========================================================================== 
 * 巡线可调参数
 * 红外决策频率为 400Hz：1个周期 = 2.5ms。
 * ========================================================================== */
float Turn90Angle  = 135.0f;
float TurnMaxAngle = 60.0f;
float TurnMidAngle = 50.0f;
float TurnMinAngle = 20.0f;

float BaseSpeed    = 300.0f; /* mm/s */
float BigTurnBaseSpeed = 220.0f; /* 大弯时轮速约为160/280mm/s */
/* 普通弯道的降速尺度；数值越大，弯道保留的前进速度越多。 */
float ForwardLimit = 600.0f;

uint32_t TurnStraightCycles   = 0U;   /* 直角确认后立即转向，不再强制前冲 */
uint32_t TurnHoldMaxCycles    = 400U; /* 最长保持直角转向1.0s */
uint32_t TurnConfirmCycles    = 2U;   /* 滤波后再确认5ms，抑制误触发 */
uint32_t TurnMinHoldCycles    = 12U;  /* 转向立即开始，但30ms内不允许误退出 */
uint32_t TurnExitStraightCycles = 2U; /* 居中持续5ms后退出锁定 */

uint32_t IRFilterSamples = 3U;       /* 同一原始状态连续3次才采用 */
uint32_t LostSearchCycles = 640U;    /* 丢线后沿最近转向搜索1600ms */

float WheelAccelLimit = 1000.0f;     /* mm/s^2 */
float WheelDecelLimit = 1800.0f;     /* mm/s^2 */
float WheelSteerResponseLimit = 12000.0f; /* 转向建立/消除速度，mm/s^2 */

typedef enum {
    STATE_CROSS         = 0x00, /* 四路都在黑线上 */
    STATE_LEFT_90_A     = 0x01,
    STATE_LEFT_90_B     = 0x03,
    STATE_RIGHT_90_A    = 0x08,
    STATE_RIGHT_90_B    = 0x0C,
    STATE_LEFT_BIG      = 0x07,
    STATE_RIGHT_BIG     = 0x0E,
    STATE_LEFT_SMALL    = 0x0B,
    STATE_RIGHT_SMALL   = 0x0D,
    STATE_STRAIGHT      = 0x09,
    STATE_LOST          = 0x0F  /* 四路都在白色区域 */
} SensorState_t;

float base_speed_mm = 0.0f;
float turn_diff = 0.0f;
static volatile uint8_t g_current_mode = STATE_LOST;

uint8_t IR_GetCurrentMode(void)
{
    return g_current_mode;
}

const char *IR_GetModeName(uint8_t mode)
{
    switch (mode) {
        case STATE_CROSS:         return "CROSS";
        case STATE_LEFT_90_A:
        case STATE_LEFT_90_B:     return "LEFT_90";
        case STATE_RIGHT_90_A:
        case STATE_RIGHT_90_B:    return "RIGHT_90";
        case STATE_LEFT_BIG:      return "LEFT_BIG";
        case STATE_RIGHT_BIG:     return "RIGHT_BIG";
        case STATE_LEFT_SMALL:    return "LEFT_SMALL";
        case STATE_RIGHT_SMALL:   return "RIGHT_SMALL";
        case STATE_STRAIGHT:      return "STRAIGHT";
        case STATE_LOST:          return "LOST";
        default:                  return "ADJUST";
    }
}

static bool Is90DegreeState(uint8_t state)
{
    return state == STATE_LEFT_90_A || state == STATE_LEFT_90_B ||
           state == STATE_RIGHT_90_A || state == STATE_RIGHT_90_B;
}

static float SpeedRamp_Update(float current, float target,
                              float accel_rate, float decel_rate)
{
    float rate = accel_rate;
    if (fabsf(target) < fabsf(current) || current * target < 0.0f) {
        rate = decel_rate;
    }

    float max_step = rate / (float)CONTROL_FREQUENCY;
    float delta = target - current;
    if (delta > max_step)  return current + max_step;
    if (delta < -max_step) return current - max_step;
    return target;
}

static uint8_t ReadRawSensorState(void)
{
    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1U : 0U;
    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1U : 0U;

    /* 实车第1、2路接线与原标号相反；黑线=0，白色反射=1。 */
    ir_dh1_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1U : 0U;
    ir_dh2_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1U : 0U;

    return (uint8_t)((ir_dh1_state << 3) | (ir_dh2_state << 2) |
                     (ir_dh3_state << 1) | ir_dh4_state);
}

static uint8_t FilterSensorState(uint8_t raw)
{
    static uint8_t last_raw = 0xFFU;
    static uint8_t stable = STATE_LOST;
    static uint32_t same_count = 0U;

    if (raw == last_raw) {
        if (same_count < UINT32_MAX) same_count++;
    } else {
        last_raw = raw;
        same_count = 1U;
    }

    uint32_t required = IRFilterSamples > 0U ? IRFilterSamples : 1U;
    if (same_count >= required) stable = raw;
    return stable;
}

/* 未在原示例状态表中的组合，根据所有压黑线探头的位置计算方向。
 * 逻辑顺序从左到右为 DH1..DH4，左侧黑线产生正转向量。 */
static float TurnFromWeightedState(uint8_t state)
{
    static const float weight[4] = { 3.0f, 1.0f, -1.0f, -3.0f };
    float sum = 0.0f;
    uint32_t black_count = 0U;

    for (uint32_t i = 0U; i < 4U; i++) {
        uint8_t mask = (uint8_t)(1U << (3U - i));
        if ((state & mask) == 0U) {
            sum += weight[i];
            black_count++;
        }
    }

    if (black_count == 0U || black_count == 4U) return 0.0f;
    float position = sum / (float)black_count;
    return TurnMinAngle * position;
}

static float TurnFromState(uint8_t state)
{
    switch (state) {
        case STATE_CROSS:
        case STATE_STRAIGHT:    return 0.0f;
        case STATE_LEFT_90_A:
        case STATE_LEFT_90_B:   return Turn90Angle;
        case STATE_RIGHT_90_A:
        case STATE_RIGHT_90_B:  return -Turn90Angle;
        case STATE_LEFT_BIG:    return TurnMaxAngle;
        case STATE_RIGHT_BIG:   return -TurnMaxAngle;
        case STATE_LEFT_SMALL:  return TurnMinAngle;
        case STATE_RIGHT_SMALL: return -TurnMinAngle;
        default:                return TurnFromWeightedState(state);
    }
}

void IRDM_line_inspection(void)
{
    static float last_search_turn = 0.0f;
    static uint8_t turn_candidate = STATE_STRAIGHT;
    static uint8_t saved_turn_state = STATE_STRAIGHT;
    static uint32_t turn_confirm_count = 0U;
    static uint32_t turn_count = 0U;
    static uint32_t exit_straight_count = 0U;
    static uint32_t lost_count = 0U;
    static float ramp_left_mmps = 0.0f;
    static float ramp_right_mmps = 0.0f;

    uint8_t sensor_state = FilterSensorState(ReadRawSensorState());

    /* 直角弯必须持续若干周期，避免单次毛刺触发长时间锁定。 */
    if (turn_count == 0U && Is90DegreeState(sensor_state)) {
        if (sensor_state == turn_candidate) {
            turn_confirm_count++;
        } else {
            turn_candidate = sensor_state;
            turn_confirm_count = 1U;
        }

        uint32_t required = TurnConfirmCycles > 0U ? TurnConfirmCycles : 1U;
        if (turn_confirm_count >= required) {
            saved_turn_state = sensor_state;
            turn_count = 1U;
            turn_confirm_count = 0U;
            exit_straight_count = 0U;
        }
    } else if (!Is90DegreeState(sensor_state)) {
        turn_confirm_count = 0U;
    }

    if (turn_count > 0U) {
        if (turn_count < TurnStraightCycles) {
            sensor_state = STATE_STRAIGHT;
        } else {
            if (turn_count >= TurnStraightCycles + TurnMinHoldCycles &&
                sensor_state == STATE_STRAIGHT) exit_straight_count++;
            else exit_straight_count = 0U;

            if (turn_count >= TurnHoldMaxCycles ||
                exit_straight_count >= TurnExitStraightCycles) {
                turn_count = 0U;
                exit_straight_count = 0U;
            } else {
                sensor_state = saved_turn_state;
                turn_count++;
            }
        }

        if (turn_count > 0U && turn_count < TurnStraightCycles) turn_count++;
    }

    bool lost_stop = false;
    g_current_mode = sensor_state;
    if (sensor_state == STATE_LOST) {
        lost_count++;
        if (lost_count <= LostSearchCycles) {
            /* 用最近一次非零转向的方向找线；幅度限制为TurnMidAngle，
             * 避免沿直角弯的大转向量继续向弯内扎。 */
            turn_diff = last_search_turn;
        } else {
            turn_diff = 0.0f;
            lost_stop = true;
        }
    } else {
        lost_count = 0U;
        turn_diff = TurnFromState(sensor_state);
        if (turn_diff > 0.0f) last_search_turn = TurnMidAngle;
        else if (turn_diff < 0.0f) last_search_turn = -TurnMidAngle;
    }

    if (lost_stop || ForwardLimit <= 0.0f) {
        base_speed_mm = 0.0f;
    } else if (Is90DegreeState(sensor_state)) {
        /* 直角弯采用单轮转向：内侧轮=0，外侧轮=2*Turn90Angle。
         * 左转：left=0/right>0；右转：right=0/left>0。 */
        base_speed_mm = fabsf(turn_diff);
    } else if (sensor_state == STATE_LEFT_BIG ||
               sensor_state == STATE_RIGHT_BIG) {
        /* 大弯使用独立的较高基础速度。 */
        base_speed_mm = BigTurnBaseSpeed;
    } else if (fabsf(turn_diff) < ForwardLimit) {
        base_speed_mm = BaseSpeed * (1.0f - fabsf(turn_diff) / ForwardLimit);
    } else {
        base_speed_mm = 0.0f;
    }

    float target_left_mmps  = base_speed_mm - turn_diff;
    float target_right_mmps = base_speed_mm + turn_diff;

    if (Flag_Stop) {
        ramp_left_mmps = 0.0f;
        ramp_right_mmps = 0.0f;
    } else {
        /*
         * 直线启停继续使用柔和斜坡；转向量发生变化时提高左右轮响应。
         * 否则直角弯退出后，内侧轮从0恢复到直线速度约需0.3s，
         * 这段残余差速会让车头继续转动并产生明显过冲。
         */
        float current_turn = 0.5f * (ramp_right_mmps - ramp_left_mmps);
        bool steering_transition =
            fabsf(turn_diff - current_turn) > 0.5f;
        float accel_rate = steering_transition ? WheelSteerResponseLimit
                                               : WheelAccelLimit;
        float decel_rate = steering_transition ? WheelSteerResponseLimit
                                               : WheelDecelLimit;

        ramp_left_mmps = SpeedRamp_Update(ramp_left_mmps, target_left_mmps,
                                          accel_rate, decel_rate);
        ramp_right_mmps = SpeedRamp_Update(ramp_right_mmps, target_right_mmps,
                                           accel_rate, decel_rate);
    }

    MotorA.Target_Encoder = 0.001f * ramp_left_mmps;
    MotorB.Target_Encoder = 0.001f * ramp_right_mmps;
}

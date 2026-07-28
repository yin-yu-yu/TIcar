/**
 * @file    robot_config.h
 * @brief   机器人机械参数与物理常量
 *
 * 所有可调机器人参数均集中在此处。
 * 源文件中不得散落未经定义的常量。
 *
 * 用法：#include "robot_config.h"
 */

#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 1. 底盘类型
 * ======================================================================== */
typedef enum {
    CHASSIS_MECANUM      = 0,
    CHASSIS_OMNI         = 1,
    CHASSIS_ACKERMANN    = 2,
    CHASSIS_DIFFERENTIAL = 3,
    CHASSIS_4WD          = 4,
    CHASSIS_TANK         = 5,
} ChassisType_t;

#define CHASSIS_TYPE    CHASSIS_DIFFERENTIAL

/* ========================================================================
 * 2. 车轮与机械参数
 * ======================================================================== */
#define WHEEL_DIAMETER_MM       66.5f       /* 车轮直径（mm）                    */
#define WHEEL_PERIMETER_M       0.20891591f /* 车轮周长（m）                     */
#define WHEEL_SPACING_M         0.2100f     /* 两轮中心间距（m）                 */
#define MOTOR_GEAR_RATIO        28.0f       /* 电机减速箱减速比                  */
#define ENCODER_PPR             13.0f       /* 暂按原工程编码器每转脉冲数        */
#define ENCODER_MULTIPLES       2           /* 编码器正交倍频系数                */

/* ========================================================================
 * 3. 运动限制
 * ======================================================================== */
#define MAX_LINEAR_SPEED_MPS    0.5f        /* 最大前进/后退速度（m/s）          */
#define MAX_ANGULAR_SPEED_RPS   1.5f        /* 最大角速度（rad/s）               */
#define TURN_RADIUS_MIN_M       0.3f        /* 最小转弯半径（受万向轮限制）      */

/* ========================================================================
 * 4. PWM 参数
 * ======================================================================== */
#define PWM_PERIOD              8000        /* PWM 定时器周期                   */
#define PWM_MAX                 8000        /* 最大 PWM 占空比数值              */
#define PWM_DEAD_ZONE           0           /* 电机死区（0 表示无死区）          */

/* ========================================================================
 * 5. 控制循环
 * ======================================================================== */
#define CONTROL_FREQ_HZ         400         /* 主控制循环频率                   */
#define CONTROL_PERIOD_MS       2.5f        /* 1000 / CONTROL_FREQ_HZ           */
#define CONTROL_DT_S            0.0025f     /* 控制周期（秒）                   */

/* ========================================================================
 * 6. 电池监测
 * ======================================================================== */
#define BATT_VOLTAGE_DIVIDER     11.0f      /* ADC 分压比                       */
#define BATT_ADC_REF_V           3.3f       /* ADC 参考电压                     */
#define BATT_ADC_RESOLUTION      4096.0f    /* 12 位 ADC                        */
#define BATT_WARN_THRESHOLD_V    7.0f       /* 低电量警告阈值（V）              */
#define BATT_CRITICAL_THRESHOLD_V 6.5f      /* 严重低电量阈值（V）              */
#define BATT_FULL_V              8.4f       /* 满电电压（2S 锂电池）            */

/* ========================================================================
 * 7. 红外循迹传感器
 * ======================================================================== */
#define IR_SENSOR_COUNT          4          /* 红外循迹传感器数量                */

#ifdef __cplusplus
}
#endif

#endif /* _ROBOT_CONFIG_H_ */

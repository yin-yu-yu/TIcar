/**
 * @file    kinematics.h
 * @brief   差速驱动运动学（正运动学与逆运动学）
 *
 * 逆运动学：ChassisCmd → WheelSpeed
 * 正运动学：WheelSpeed → ChassisCmd（供里程计使用）
 *
 * 模型：前万向轮、双轮差速驱动。
 *       不支持原地转向，转向时必须保持 vx != 0。
 */

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * 类型定义
 * ======================================================================== */

/** 底盘级运动命令（车体坐标系） */
typedef struct {
    float vx;    /* 前向速度（m/s），正值为前进                      */
    float wz;    /* 角速度（rad/s），正值为逆时针（左转）            */
} ChassisCmd_t;

/** 各轮速度输出 */
typedef struct {
    float left;   /* 左轮速度（m/s）          */
    float right;  /* 右轮速度（m/s）          */
} WheelSpeed_t;

/* ========================================================================
 * 公共函数
 * ======================================================================== */

/**
 * @brief  使用机器人几何参数初始化运动学模块
 * @note   参数读取自 robot_config.h
 */
void Kinematics_Init(void);

/**
 * @brief  逆运动学：由底盘命令计算车轮速度
 * @param  cmd  期望底盘运动（vx、wz）
 * @return      所需车轮速度（m/s）
 *
 * 公式：
 *   V_left  = vx - wz * WheelSpacing/2
 *   V_right = vx + wz * WheelSpacing/2
 *
 * 约束：
 *   - 强制使用最小转弯半径（TURN_RADIUS_MIN_M）
 *   - 将输出速度限制在 MAX_LINEAR_SPEED_MPS 内
 */
WheelSpeed_t Kinematics_Inverse(ChassisCmd_t cmd);

/**
 * @brief  正运动学：由车轮速度计算底盘运动
 * @param  wheels  测得的车轮速度（m/s）
 * @return         估计的底盘运动
 *
 * 公式：
 *   vx = (V_left + V_right) / 2
 *   wz = (V_right - V_left) / WheelSpacing
 */
ChassisCmd_t Kinematics_Forward(WheelSpeed_t wheels);

/**
 * @brief  计算给定速度下的最小转弯半径
 * @param  vx  前向速度（m/s）
 * @return     可达到的最小转弯半径（m）
 *
 * 对前万向轮差速驱动：
 *   R = max(vx/wz_max, TURN_RADIUS_MIN_M)
 */
float Kinematics_MinTurnRadius(float vx);

#ifdef __cplusplus
}
#endif

#endif /* _KINEMATICS_H_ */

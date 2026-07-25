# CHASSIS API — 小车底盘接口文档

> 版本: v1.0 | 更新: 2026-07-25

本文档定义小车底盘工程各层模块的完整 API。

---

## 1. 分层架构

```
Application  ← SM_Run(), BT_Protocol_Handler()
Middleware   ← PID_Compute(), Kinematics_Inverse()
Hardware     ← Motor_SetPWM(), Encoder_GetCount()
BSP          ← SYSCFG_DL_init() (SysConfig)
```

**调用规则：**
- 上层可调用下层，下层不可调用上层
- 同层模块之间解耦，通过上层协调
- TIMER_0 ISR (200Hz) 中调用实时控制，main loop 中调用非实时

---

## 2. Hardware 层 API

### 2.1 motor.h — 电机驱动

| 函数 | 说明 |
|------|------|
| `void Motor_SetPWM(int16_t left, int16_t right)` | 设置左右电机 PWM（±8000，正=前进） |
| `void Motor_Stop(void)` | 停止（滑行） |
| `void Motor_Brake(void)` | 刹车（短接制动） |

**宏兼容：** `Set_PWM(l, r)` → `Motor_SetPWM(l, r)`

### 2.2 encoder.h — 编码器

| 函数/变量 | 说明 |
|------|------|
| `int32_t Get_Encoder_countA` | 编码器 A 累计脉冲（ISR 累加） |
| `int32_t Get_Encoder_countB` | 编码器 B 累计脉冲 |
| `int32_t Encoder_GetCountA(void)` | 读取编码器 A |
| `int32_t Encoder_GetCountB(void)` | 读取编码器 B |
| `void Encoder_Reset(void)` | 清零 |

### 2.3 mpu6050.h — IMU (STUB)

| 函数 | 说明 | 状态 |
|------|------|------|
| `void MPU6050_Init(void)` | 初始化 I2C + DMP | STUB |
| `bool MPU6050_DataReady(void)` | 数据就绪 | STUB |
| `void MPU6050_Read(float *g, *a, *ang)` | 读取全部 | STUB |
| `float MPU6050_GetYaw(void)` | Z 轴偏航角 (°) | STUB |
| `float MPU6050_GetPitch(void)` | 俯仰角 (°) | STUB |
| `float MPU6050_GetRoll(void)` | 横滚角 (°) | STUB |
| `float MPU6050_GetGyroZ(void)` | Z 轴角速度 (°/s) | STUB |

### 2.4 ir_track.h — 四路循迹

| 函数 | 说明 |
|------|------|
| `uint8_t IR_GetSensorState(void)` | 4-bit 传感器状态 (bit3=DH1 ... bit0=DH4) |
| `float IR_GetPositionError(void)` | 位置偏差 (mm) |
| `void IR_LineDetect_Update(void)` | 巡线状态机更新 (200Hz) |
| `void IR_SetBaseSpeed(float mmps)` | 设置基准速度 |
| `float IR_GetTurnDiff(void)` | 当前转弯差值 |

### 2.5 uart_debug.h — 调试串口

| 函数 | 说明 |
|------|------|
| `void Debug_Printf(const char *fmt, ...)` | printf 风格输出 |
| `void Debug_SendBinary(const uint8_t *d, uint16_t n)` | 二进制发送 |
| `void Debug_PutChar(char ch)` | 发送单字符 |

### 2.6 uart_bt.h — 蓝牙串口

| 函数 | 说明 |
|------|------|
| `void BT_Init(void)` | 初始化 DMA 接收 |
| `bool BT_DataAvailable(void)` | 有新数据？ |
| `uint8_t BT_ReadByte(void)` | 读一字节 |
| `void BT_SendBytes(const uint8_t *d, uint16_t n)` | 发送字节 |
| `void BT_Printf(const char *fmt, ...)` | printf 风格输出 |
| `void BT_DMAConfig(void)` | 配置 DMA 接收 |

### 2.7 adc.h — 电池电压

| 函数 | 说明 |
|------|------|
| `float Get_battery_volt(void)` | 返回电池电压 (V) |
| `float Batt_GetVoltage(void)` | 同上 (新命名风格) |

### 2.8 oled.h — 显示 (新增辅助函数建议)

| 函数 | 说明 |
|------|------|
| `void OLED_Init(void)` | 初始化 |
| `void OLED_ShowString(x, y, str)` | 显示字符串 |
| `void OLED_ShowNumber(x, y, num, len, size)` | 显示数字 |
| `void OLED_Refresh_Gram(void)` | 刷新屏幕 |
| *(建议队员添加)* `OLED_ShowBattery(float v)` | 电池图标 |
| *(建议队员添加)* `OLED_ShowWarning(const char *m)` | 警告信息 |
| *(建议队员添加)* `OLED_ShowState(const char *s)` | 状态名 |

---

## 3. Middleware 层 API

### 3.1 pid.h — PID 控制器

```c
typedef struct { float Kp, Ki, Kd; float setpoint, integral, prev_error, prev_prev_error;
                 float out_min, out_max, output; } PID_t;
```

| 函数 | 说明 |
|------|------|
| `void PID_Init(PID_t *p, float kp, ki, kd, min, max)` | 初始化 |
| `void PID_SetSetpoint(PID_t *p, float sp)` | 设目标值 |
| `float PID_Compute(PID_t *p, float measured, float dt)` | 计算 (增量式) |
| `void PID_Reset(PID_t *p)` | 清零 |
| `void PID_SetGains(PID_t *p, float kp, ki, kd)` | 在线调参 |

**公式：** `output += Kp*(e(k)-e(k-1)) + Ki*e(k)*dt + Kd*(e(k)-2e(k-1)+e(k-2))/dt`

### 3.2 kinematics.h — 运动学

```c
typedef struct { float vx; float wz; }         ChassisCmd_t;    // 线速度(m/s) + 角速度(rad/s)
typedef struct { float left; float right; }    WheelSpeed_t;    // 左右轮速度(m/s)
```

| 函数 | 说明 |
|------|------|
| `void Kinematics_Init(void)` | 初始化（读取 robot_config） |
| `WheelSpeed_t Kinematics_Inverse(ChassisCmd_t cmd)` | 逆运动学：底盘 → 轮速 |
| `ChassisCmd_t Kinematics_Forward(WheelSpeed_t ws)` | 正运动学：轮速 → 底盘 |
| `float Kinematics_MinTurnRadius(float vx)` | 最小转弯半径 |

**逆运动学公式（差速）：**
```
V_left  = vx - wz * WheelSpacing/2
V_right = vx + wz * WheelSpacing/2
```
- 输出受 `MAX_LINEAR_SPEED_MPS` 限幅
- 转弯半径受 `TURN_RADIUS_MIN_M` 约束

### 3.3 odometry.h — 里程计

```c
typedef struct { float x, y, theta; float vx, vz; } Odom_t;
```

| 函数 | 说明 |
|------|------|
| `void Odom_Init(void)` | 初始化（原点） |
| `void Odom_Update(int32_t encL, int32_t encR, float dt)` | 更新（200Hz） |
| `Odom_t Odom_GetPose(void)` | 获取位姿 |
| `void Odom_Reset(void)` | 复位 |
| `float Odom_GetDistance(void)` | 总里程 (m) |

### 3.4 filter.h — 滤波器

| 函数 | 说明 |
|------|------|
| `float Filter_LowPass(float in, float *prev, float alpha)` | 一阶低通 |
| `float Filter_Complementary(float acc, float gyro, float dt, float a, float *fused)` | 互补滤波 |
| `float Filter_RateLimit(float tgt, float cur, float rate, float dt)` | 速率限制 |
| `float Filter_DeadZone(float val, float zone)` | 死区 |

---

## 4. Application 层 API

### 4.1 state_machine.h — 状态机 (队长负责)

```c
typedef enum {
    STATE_INIT, STATE_IDLE, STATE_RC_DRIVE,
    STATE_LINE_FOLLOW, STATE_LOW_BATTERY, STATE_ERROR
} RobotState_t;
```

| 函数 | 说明 |
|------|------|
| `void SM_Init(void)` | 初始化 |
| `void SM_Run(void)` | 运行一次 (200Hz) |
| `void SM_Transition(RobotState_t next)` | 状态切换 |
| `RobotState_t SM_GetState(void)` | 当前状态 |
| `const char* SM_GetStateName(RobotState_t s)` | 状态名 |
| `bool SM_IsMoving(void)` | 是否运动中 |
| `void SM_ReportBattery(float v)` | 电池电压上报 |

**状态转换图：**
```
INIT → IDLE ⇄ RC_DRIVE
            ⇄ LINE_FOLLOW
            ⇄ LOW_BATTERY
任何状态 → ERROR (致命故障)
```

### 4.2 motion_control.h — 运动控制 (队员)

| 函数 | 说明 |
|------|------|
| `void MotionControl_Init(void)` | 创建 PID 实例 |
| `void MotionControl_SetTarget(ChassisCmd_t cmd)` | 设目标速度 |
| `void MotionControl_Update(void)` | 编码器→PID→PWM (200Hz) |
| `void MotionControl_Stop(void)` | 急停 + 复位 PID |

### 4.3 line_follow.h — 巡线 (队员)

| 函数 | 说明 |
|------|------|
| `void LineFollow_Init(void)` | 初始化 |
| `float LineFollow_ComputeCorrection(uint8_t state)` | 计算修正值 |
| `void LineFollow_SetBaseSpeed(float mps)` | 设基准速度 |

### 4.4 bt_protocol.h — 蓝牙协议 (队员)

| 函数 | 说明 |
|------|------|
| `void BT_Protocol_Init(void)` | 初始化 |
| `void BT_Protocol_Handler(void)` | 处理 DMA 缓冲区 (主循环) |
| `void BT_Protocol_SendStatus(RobotState_t, float batt, float sl, float sr)` | 上报状态 |
| `void BT_Protocol_SendPID(void)` | 上报 PID 参数 |

---

## 5. 数据流 & 时序

```
main() — 低频率 (非实时)
  ├─ Batt_GetVoltage()
  ├─ SM_ReportBattery()
  ├─ BT_Protocol_Handler()  ← DMA 数据帧解析
  ├─ APP_Show()              ← 蓝牙状态上报
  └─ OLED 刷新

TIMER0_ISR — 200Hz (5ms 周期)
  ├─ Encoder_GetCount()      ← 读编码器脉冲
  ├─ MPU6050_Read()          ← IMU 数据 (STUB)
  ├─ Odom_Update()           ← 里程计更新
  ├─ SM_Run()                ← 状态机决策
  │    ├─ [RC_DRIVE]     读取 BT 标志 → MotionControl_SetTarget()
  │    ├─ [LINE_FOLLOW]  IR_LineDetect_Update()
  │    └─ [LOW_BATTERY]  LED_Flash()
  ├─ MotionControl_Update()  ← PID → Motor_SetPWM()
  └─ (return)
```

---

## 6. 配置文件

### robot_config.h

| 定义 | 默认值 | 说明 |
|------|--------|------|
| `WHEEL_DIAMETER_MM` | 65.0 | 轮径 mm |
| `WHEEL_SPACING_M` | 0.161 | 轮距 m |
| `MOTOR_GEAR_RATIO` | 28.0 | 减速比 |
| `ENCODER_PPR` | 13.0 | 编码器线数 |
| `CONTROL_FREQ_HZ` | 200 | 控制频率 |
| `PWM_MAX` | 8000 | PWM 最大值 |
| `MAX_LINEAR_SPEED_MPS` | 0.5 | 最大线速度 |
| `TURN_RADIUS_MIN_M` | 0.3 | 最小转弯半径 |
| `BATT_WARN_THRESHOLD_V` | 7.0 | 低电量警告 |
| `BATT_CRITICAL_THRESHOLD_V` | 6.5 | 严重低电量 |

### pid_config.h

| 定义 | 默认值 | 用途 |
|------|--------|------|
| `VELOCITY_KP_DEFAULT` | 400.0 | 速度 P 增益 |
| `VELOCITY_KI_DEFAULT` | 400.0 | 速度 I 增益 |
| `VELOCITY_OUT_MAX` | 7800.0 | 速度 PID 上限 |
| `ANGLE_KP_DEFAULT` | 50.0 | 角度 PID |
| `LINE_KP_DEFAULT` | 30.0 | 巡线 PID |

---

## 7. 遗留模块兼容说明

以下原有模块尚未完成迁移，仍然可用但请勿新增对其的依赖：

| 模块 | 路径 | 迁移目标 |
|------|------|---------|
| PI 控制 | `Control/control.c` | `Middleware/pid.c` + `Application/motion_control.c` |
| 巡线 | `Hardware/IR_Module.c` | `Application/line_follow.c` |
| 蓝牙解析 | `Control/uart_callback.c` | `Application/bt_protocol.c` |
| APP 显示 | `Control/show.c` | 保留或整合入 `bt_protocol.c` |
| 虚拟示波器 | `Control/DataScope_DP.C` | `Application/debug_scope.c` |

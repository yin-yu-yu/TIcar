# WHEELTEC C07A 小车底盘工程

## 工程信息

| 项目 | 详情 |
|------|------|
| MCU | MSPM0G3507 (Cortex-M0+, 80MHz, 128KB Flash, 32KB SRAM) |
| 编译器 | TI Arm Clang 5.1.1.LTS |
| IDE | Code Composer Studio (CCS) Theia |
| SDK | MSPM0 SDK v2.11.00.07 |
| 配置工具 | SysConfig 1.28.0 |
| 底盘类型 | **差速驱动**（后轮驱动 + 前轮万向轮） |
| 调试器 | XDS110 (板载) |

## 底盘运动特性

- 后轮双电机差速驱动，前轮为被动万向轮
- **无法原地自旋** — 转弯必须伴随前进/后退运动
- 运动模式：左前转、右前转、左后转、右后转
- 最小转弯半径：~0.3m（受万向轮限制）

## 目录结构

```
project/
├── README.md                    ← 你在这里
├── empty.c                      # 主函数 (main) + CHASSIS_DEBUG 开关
├── empty.h                      # 顶层 include 汇总
├── empty.syscfg                 # SysConfig 硬件配置
│
├── Config/                      # 参数配置文件
│   ├── robot_config.h           # 小车机械参数/运动限制
│   └── pid_config.h             # PID 参数预设
│
├── Hardware/                    # 硬件驱动层
│   ├── board.c/h                # 板级支持 (SysTick/延时/printf)
│   ├── motor.c/h                # 电机 PWM + 方向控制
│   ├── encoder.c/h              # 编码器正交解码
│   ├── mpu6050.c/h              # MPU6050 IMU (Mahony AHRS, 已完成)
│   ├── bsp_siic.c/h             # 软件 I2C 驱动 (PA0=SDA, PA1=SCL)
│   ├── ir_track.c/h             # 四路循迹驱动 (从 IR_Module 迁移)
│   ├── IR_Module.c/h            # 四路循迹 (遗留，待废弃)
│   ├── uart_debug.c/h           # 调试串口 UART0
│   ├── uart_bt.c/h              # 蓝牙串口 UART1 + DMA
│   ├── adc.c/h                  # 电池电压 ADC
│   ├── oled.c/h, oledfont.h     # OLED 128x64 显示
│   ├── key.c/h                  # 按键驱动
│   └── led.c/h                  # LED 驱动
│
├── Middleware/                  # 中间层 (算法/控制)
│   ├── pid.c/h                  # 通用增量式 PID
│   ├── kinematics.c/h           # 差速运动学正逆解
│   ├── odometry.c/h             # 里程计
│   └── filter.c/h               # 滤波器
│
├── Application/                 # 应用层
│   ├── state_machine.c/h        # 总体状态机 (队长负责)
│   ├── motion_control.c/h       # 运动控制 (队员填充)
│   ├── line_follow.c/h          # 巡线策略 (队员填充)
│   ├── bt_protocol.c/h          # 蓝牙协议 (队员填充)
│   └── debug_scope.c/h          # 虚拟示波器
│
├── Control/                     # 遗留模块 (待迁移)
│   ├── control.c/h              # 原始 PI 控制 + 运动学
│   ├── show.c/h                 # OLED + APP 显示
│   ├── uart_callback.c/h        # 蓝牙命令解析
│   └── DataScope_DP.c/h         # 虚拟示波器协议
│
├── Debug/                       # SysConfig 生成 (只读，勿手动修改)
│   └── ti_msp_dl_config.c/h
│
└── docs/
    └── CHASSIS_API.md            # 接口文档
```

## 各模块状态

> 详见 [`TODO.md`](TODO.md) — 队员进度跟踪

| 模块 | 状态 | 负责人 | 备注 |
|------|------|--------|------|
| 状态机 | 🔶 基本完成 | **队长** | 2 个 TODO 待完善 |
| 运动控制 PID | ✅ 已完成 | 队员 | 速度闭环完整实现 |
| 巡线策略 | ✅ 已完成 | 队员 | 从 IR_Module 迁移完毕 |
| 蓝牙协议 | ✅ 已完成 | 队员 | 从 uart_callback 迁移完毕 |
| 虚拟示波器 | ✅ 已完成 | — | DataScope 协议封装 |
| 电机 + PWM | ✅ 已完成 | 队员 | 接口已标准化 |
| 编码器 | ✅ 已完成 | 队员 | 接口已标准化 |
| **MPU6050** | **✅ 已完成** | **队员 D** | Mahony AHRS + 软件 I2C |
| 四路循迹 | ✅ 已完成 | 队员 | IR_Module → ir_track |
| 调试串口 | ✅ 已完成 | — | UART0 |
| OLED 显示 | ✅ 已完成 | — | 状态/电压/速度 |
| PID 控制 | ✅ 已完成 | — | Middleware/pid.c |
| 运动学 | ✅ 已完成 | — | Middleware/kinematics.c |
| 里程计 | ✅ 已完成 | — | Middleware/odometry.c |

## 调试开关 (CHASSIS_DEBUG)

工程支持两种编译模式，在 `empty.c` 第 18 行控制：

```c
#define CHASSIS_DEBUG    // 取消注释 = 调试模式

#ifdef CHASSIS_DEBUG
    // 调试循环: 13 项接口自检（电机/编码器/IR/PID/运动学/里程计/...）
    //           自检通过后进入 while(1) 主循环
    //           各队员可在调试区添加测试代码
#else
    // 比赛循环: 完整状态机运行 (TIMER ISR 200Hz 实时控制)
#endif
```

**调试模式流程：**
1. 上电后依次执行 13 项接口自检（通过串口输出 PASS/SKIP）
2. 自检通过后进入 `while(1)` 主循环
3. 队员在各自调试区添加测试代码，互不干扰

**多人协作流程：**
1. 队员调试时取消注释 `CHASSIS_DEBUG`，在调试段加自己的测试代码
2. 所有队员测试完毕后注释掉，切换到比赛模式
3. 不要在调试段之外修改共享模块，避免 Git 冲突

---

## AI Agent 提示词（给队员使用）

> 队员可将对应模块的提示词直接粘贴给自己的 AI agent。

### 模块 1: MPU6050 移植 ✅ 已完成

```
✅ 已从 WHEELTEC_C07A_BalanceCar 移植完成。

实现方案: Mahony AHRS（四元数 + PI 修正），无需 DMP 固件依赖

移植文件:
- Hardware/bsp_siic.c/h  — 软件 I2C 抽象层 (PA0=SDA, PA1=SCL)
- Hardware/mpu6050.c/h   — MPU6050 驱动 + Mahony AHRS 姿态解算

接口状态:
- MPU6050_Init()           ✅ I2C 初始化 + WHO_AM_I 验证 + 寄存器配置
- MPU6050_DataReady()      ✅ 读取成功后返回 true
- MPU6050_Read()           ✅ 读陀螺+加计 → Mahony AHRS → 欧拉角
- MPU6050_GetYaw/Pitch/Roll() ✅ 返回 Mahony 解算的欧拉角
- MPU6050_GetGyroZ()       ✅ 返回 Z 轴角速度 (dps)

配置: 陀螺 ±2000dps, 加计 ±2g, DLPF 44Hz, 采样率 200Hz
验证: empty.c 测试 13 中完成接口自检
```

### 模块 2: 电机驱动标准化

```
确保 Hardware/motor.c 接口正确:

1. Motor_SetPWM(+2000, +2000) → 小车直线前进
2. 方向不对则交换 AIN1/AIN2 或 BIN1/BIN2 逻辑
3. 如有死区 (低速不动)，在 Config/robot_config.h 的 PWM_DEAD_ZONE 配置

调试:
  Motor_SetPWM(2000, 2000); delay_ms(500);  // 前进
  Motor_Stop(); delay_ms(500);               // 停止
  Motor_SetPWM(-2000, -2000); delay_ms(500); // 后退
```

### 模块 3: 巡线策略

```
将 Hardware/IR_Module.c 中的 IRDM_line_inspection() 逻辑迁移到 Application/line_follow.c。

```

### 模块 4: 蓝牙协议

```
将 Control/uart_callback.c 中的命令解析逻辑迁移到 Application/bt_protocol.c。

协议 (WheelTec APP 兼容):
- 方向: 0x41~0x48 → 8 方向摇杆
- 转向: 0x4B → 转向模式, 0x43/0x47 → 左/右旋转
- 速度: 0x58 → +100mm/s, 0x59 → -100mm/s
- PID设置: {0x7B, 0x23, ParamID, Data..., 0x7D}
  ParamID: 0x30=KP, 0x31=KI, 0x32=Speed, 0x33~0x37=TurnAngles

状态上报: printf("{A%d:%d:%d:%d}$", speedL, speedR, batt%, 0)

参考: Control/uart_callback.c, Control/show.c APP_Show()
```

### 模块 5: 运动控制 PID

```
在 Application/motion_control.c 中实现速度闭环:

1. 创建两个 PID_t (左轮/右轮)
2. MotionControl_SetTarget(cmd): Kinematics_Inverse → PID_SetSetpoint
3. MotionControl_Update(): 读编码器速度 → PID_Compute → Motor_SetPWM

PID 默认参数: KP=400, KI=400, KD=0 (Config/pid_config.h)
控制频率: 200Hz

参考: Middleware/pid.c, Control/control.c Incremental_PI_Left/Right
```

---

## 编译 & 烧录

1. CCS Theia 打开工程文件夹
2. SysConfig 自动生成 `Debug/ti_msp_dl_config.c/h`
3. `Project → Build Project` (Ctrl+B)
4. `Run → Debug` (F11) → XDS110

## 引脚分配

| 功能 | GPIO | 备注 |
|------|------|------|
| 左电机 PWM | PB2 | TIMA1_CCP0 |
| 右电机 PWM | PB3 | TIMA1_CCP1 |
| 左电机方向 | PA13(AIN1), PA14(AIN2) | |
| 右电机方向 | PA16(BIN1), PA17(BIN2) | |
| 编码器 A/B | PA25/26, PB20/24 | GPIO 中断 |
| IR 循迹 1-4 | PA27, PA12, PB16, PB17 | |
| 蓝牙 UART | PB6(TX), PB7(RX) | UART1, 9600 |
| 调试 UART | PA10(TX), PA11(RX) | UART0, 115200 |
| OLED | PA28(SCL), PA31(SDA), PB14(RST), PB15(DC) | 软件 SPI |
| 按键 | PA18 | |
| LED | PB9 | |
| 电池 ADC | PA15 | ADC1_CH0, 11:1 分压 |
| MPU6050 I2C | PA0(SDA), PA1(SCL) | 软件 I2C |
| 额外 PWM | (待确认) | |

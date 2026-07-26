# 队员模块进度跟踪

> 最后更新：2026-07-26 | 基于代码实际审查，非 README 快照

---

## 总览

| 状态 | 数量 | 说明 |
|------|------|------|
| ✅ 已完成 | 20 | 生产就绪 |
| 🔶 基本完成 | 1 | 可用，有小 TODO |
| ❌ 待开发 | 0 | — |
| 📦 遗留（待废弃） | 4 | 功能完整，逐步迁移中 |

---

## 一、队员任务清单

### 队员 A：电机 + 编码器验证

| 子任务 | 状态 | 备注 |
|--------|------|------|
| 电机方向验证（前/后/左/右） | ✅ 已完成 | `Hardware/motor.c` — 接口已标准化 |
| 编码器计数正确性 | ✅ 已完成 | `Hardware/encoder.c` — 正交解码就绪 |
| PWM 死区参数调优 | ✅ 已完成 | `Config/robot_config.h` `PWM_DEAD_ZONE` |
| 机械参数校准（轮径/轮距/减速比） | ✅ 已完成 | `Config/robot_config.h` |
| 调试循环测试代码 | ✅ 已完成 | `empty.c` `[Member-A]` 段 |

### 队员 B：红外循迹 + 传感器

| 子任务 | 状态 | 备注 |
|--------|------|------|
| 4 路 IR 传感器读取 | ✅ 已完成 | `Hardware/ir_track.c` |
| 传感器状态分类（直线/弯道/脱线） | ✅ 已完成 | `IR_LineDetect_Update()` |
| 巡线策略迁移到 `Application/line_follow.c` | ✅ 已完成 | 从 `IR_Module.c` 迁移完毕 |
| 转弯参数调优 | ✅ 已完成 | `Config/robot_config.h` |
| 调试循环测试代码 | ✅ 已完成 | `empty.c` `[Member-B]` 段 |

### 队员 C：蓝牙协议

| 子任务 | 状态 | 备注 |
|--------|------|------|
| 协议解析迁移到 `Application/bt_protocol.c` | ✅ 已完成 | 从 `Control/uart_callback.c` 迁移完毕 |
| DMA 接收配置 | ✅ 已完成 | `BT_DMAConfig()` |
| WheelTec APP 兼容（方向/转向/速度） | ✅ 已完成 | 8方向摇杆 + 转向模式 |
| PID 参数设置帧解析 | ✅ 已完成 | `{0x7B, 0x23, ...}` 格式 |
| 状态上报 `{A...}$` | ✅ 已完成 | 速度/电量上报 |
| PID 参数上报 `{C...}$` | ✅ 已完成 | 参数查询响应 |

### 队员 D：MPU6050 移植 ✅

| 子任务 | 状态 | 备注 |
|--------|------|------|
| I2C 初始化 + 引脚配置 | ✅ 已完成 | `Hardware/bsp_siic.c/h` — 软件 I2C (PA0=SDA, PA1=SCL) |
| 传感器初始化 + WHO_AM_I 验证 | ✅ 已完成 | `MPU6050_Init()` — 陀螺 ±2000dps, 加计 ±2g, DLPF 44Hz, 采样率 200Hz |
| `MPU6050_DataReady()` 实现 | ✅ 已完成 | 读取成功后返回 true |
| `MPU6050_Read()` 实现 | ✅ 已完成 | 读取陀螺+加计原始数据，Mahony AHRS 解算姿态角 |
| `MPU6050_GetYaw/Pitch/Roll()` 实现 | ✅ 已完成 | 返回 Mahony AHRS 解算的欧拉角 |
| `MPU6050_GetGyroZ()` 实现 | ✅ 已完成 | 返回 Z 轴角速度 (dps) |
| Mahony AHRS 姿态解算 | ✅ 已完成 | 四元数 + PI 修正，无需 DMP 固件 |
| 从 WHEELTEC_C07A_BalanceCar 移植 | ✅ 已完成 | `bsp_siic.c/h` + `mpu6050.c/h` |
| 调试循环测试代码 | ✅ 已完成 | `empty.c` `[Member-D]` 段 |
| **MPU6050 模块已全部完成！** | | **Mahony AHRS，无 DMP 依赖** |

---

## 二、已完成模块清单（队长确认）

### Hardware 硬件层

| 模块 | 文件 | 状态 | 验证方法 |
|------|------|------|---------|
| 电机驱动 | `Hardware/motor.c/h` | ✅ | `Motor_SetPWM(±2000, ±2000)` |
| 编码器 | `Hardware/encoder.c/h` | ✅ | `Get_Encoder_countA/B` 读数 |
| 红外循迹 | `Hardware/ir_track.c/h` | ✅ | `IR_GetSensorState()` 4-bit |
| 蓝牙串口 | `Hardware/uart_bt.c/h` | ✅ | `BT_Printf()` + DMA 接收 |
| 调试串口 | `Hardware/uart_debug.c/h` | ✅ | `Debug_Printf()` UART0 |
| 电池 ADC | `Hardware/adc.c/h` | ✅ | `Batt_GetVoltage()` |
| OLED 显示 | `Hardware/oled.c/h` | ✅ | 128x64 软件 SPI |
| 按键 | `Hardware/key.c/h` | ✅ | 单击/双击/长按 |
| LED | `Hardware/led.c/h` | ✅ | 开关/闪烁 |
| 板级支持 | `Hardware/board.c/h` | ✅ | SysTick, delay_ms, printf |
| 软件 I2C | `Hardware/bsp_siic.c/h` | ✅ | `User_sIICDev.write/read` 抽象接口 |
| **MPU6050** | **`Hardware/mpu6050.c/h`** | **✅** | **Mahony AHRS + 软件 I2C，从 WHEELTEC 移植** |

### Middleware 中间层

| 模块 | 文件 | 状态 | 说明 |
|------|------|------|------|
| PID 控制器 | `Middleware/pid.c/h` | ✅ | 增量式 PID，防积分饱和 |
| 运动学 | `Middleware/kinematics.c/h` | ✅ | 差速正逆解 |
| 里程计 | `Middleware/odometry.c/h` | ✅ | 编码器→位姿估计 |
| 滤波器 | `Middleware/filter.c/h` | ✅ | 低通/互补/死区/速率限制 |

### Application 应用层

| 模块 | 文件 | 状态 | 负责人 | 说明 |
|------|------|------|--------|------|
| 状态机 | `Application/state_machine.c/h` | 🔶 | 队长 | 6 状态全部实现，2 个 TODO 待完善 |
| 运动控制 | `Application/motion_control.c/h` | ✅ | 队员 | PID 速度闭环完整实现 |
| 巡线策略 | `Application/line_follow.c/h` | ✅ | 队员 | 从 IR_Module 迁移完毕 |
| 蓝牙协议 | `Application/bt_protocol.c/h` | ✅ | 队员 | 从 uart_callback 迁移完毕 |
| 虚拟示波器 | `Application/debug_scope.c/h` | ✅ | 调试 | DataScope 协议封装 |

---

## 三、队长 TODO（状态机完善）

`Application/state_machine.c` 中 2 个待完善点：

- [ ] **Line 125** — `g_sm.entry_tick = 0` 替换为实际 SysTick 值，实现状态超时功能
- [ ] **Line 180** — `Handle_Init()` 中添加传感器自检（MPU6050 应答、编码器脉冲、IR 传感器状态）
- [x] **Line 77** — ~~MPU6050 移植完成后，取消 `SM_Init()` 中 `MPU6050_Init()` 的注释~~ ✅ 已完成

---

## 四、工程基础设施 TODO

- [ ] 创建 `CLAUDE.md` — 项目级 AI agent 指令文件（含架构规则、引脚分配、编译流程）
- [ ] `empty.syscfg` — 确认 MPU6050 I2C 引脚分配（当前标注"待确认"）
- [ ] `Inc/` 目录为空 — 确认是否废弃或删除
- [ ] 遗留代码 `Control/` 逐步验证后可删除（功能已全部迁移到新架构）
- [ ] `.codex/config.toml` — 填充或删除空文件

---

## 五、遗留代码迁移状态

| 旧文件 | 新位置 | 迁移状态 |
|--------|--------|---------|
| `Control/control.c/h` | `Middleware/pid.c` + `Application/motion_control.c` | ✅ 已迁移 |
| `Control/uart_callback.c/h` | `Application/bt_protocol.c` | ✅ 已迁移 |
| `Control/show.c/h` | `Application/bt_protocol.c`（状态上报） | ✅ 已迁移 |
| `Hardware/IR_Module.c/h` | `Application/line_follow.c` + `Hardware/ir_track.c` | ✅ 已迁移 |
| `Control/DataScope_DP.C/h` | `Application/debug_scope.c` | ✅ 已封装 |

---

## 六、下一步行动

### 🔴 高优先级
1. **队长：完善状态机 2 个 TODO** — 传感器自检 + 状态超时

### 🟡 中优先级
2. 创建 `CLAUDE.md` — 方便 AI agent 协作
3. 确认 MPU6050 I2C 引脚已在 `empty.syscfg` 中配置

### 🟢 低优先级
4. 清理 `Control/` 遗留代码（确认无误后）
5. 清理空目录和空文件

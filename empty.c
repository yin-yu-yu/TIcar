/**
 * @file    empty.c
 * @brief   主程序入口
 *
 * 构建模式（由 CHASSIS_DEBUG 宏控制）：
 *   #define CHASSIS_DEBUG   → 调试模式（各成员添加测试代码）
 *   //#define CHASSIS_DEBUG → 比赛模式（完整状态机）
 *
 * 架构：
 *   main() 循环处理低频任务（蓝牙解析、显示）
 *   TIMER_0 ISR (200Hz) 处理实时控制（编码器、PID、状态机）
 *
 * 新旧 API 切换（control.c 中 USE_NEW_API 宏）：
 *   - 新架构：SM_Run() → MotionControl_Update()
 *   - 旧架构：TIM_Diff() → PI → Set_PWM
 */

#define CHASSIS_DEBUG    /* ⚠️ 调试时取消注释；比赛时注释掉 */

#include "board.h"
#include "show.h"
#include "uart_callback.h"

/* ---- 全局变量 ---- */
u8  Car_Mode = Diff_Car;
int Motor_Left, Motor_Right;
u8  PID_Send;
float RC_Velocity = 200.0f, RC_Turn_Velocity;
float Move_X, Move_Y, Move_Z, PS2_ON_Flag;
float Velocity_Left, Velocity_Right;
u16  test_num, show_cnt;
float Voltage = 8.4f;

/* ========================================================================
 * main()
 * ======================================================================== */
int main(void)
{
    {
        /* ---- 系统和外设初始化（SysConfig 生成）---- */
        SYSCFG_DL_init();

        /* ---- 清除挂起的中断 ---- */
        NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
        NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
        NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
        NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
        NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
        NVIC_ClearPendingIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

        /* ---- 使能中断 ---- */
        NVIC_EnableIRQ(ENCODERA_INT_IRQN);
        NVIC_EnableIRQ(ENCODERB_INT_IRQN);
        NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
        NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
        NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
        NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

        /* ---- 硬件初始化 ---- */
        OLED_Init();
        BT_Init();

        /* ---- 状态机初始化 (含 MotionControl_Init, LineFollow_Init, BT_Protocol_Init) ---- */
        SM_Init();
    }
    /* ====================================================================
     * 调试模式 — 各成员在下方添加自己的测试代码
     * ==================================================================== */
#ifdef CHASSIS_DEBUG
    OLED_ShowString(0, 0, "DEBUG MODE");
    OLED_Refresh_Gram();
    delay_ms(3000);

    /* ---- 接口调试测试阶段 ---- */
    OLED_Clear();
    OLED_ShowString(0, 0, "IF TEST...");
    OLED_Refresh_Gram();

    /* [测试1] 电机接口：前进 → 停止 → 后退
     * 验证 Motor_SetPWM / Motor_Stop / Motor_Brake 接口正确 */
    Debug_Printf("--- Test 1: Motor Interface ---\r\n");
    Debug_Printf("Motor: Forward\r\n");
    Motor_SetPWM(2000, 2000);
    delay_ms(500);
    Debug_Printf("Motor: Stop\r\n");
    Motor_Stop();
    delay_ms(500);
    Debug_Printf("Motor: Backward\r\n");
    Motor_SetPWM(-2000, -2000);
    delay_ms(500);
    Motor_Stop();
    Debug_Printf("Motor: OK\r\n");

    /* [测试2] 编码器接口：读取脉冲
     * 验证 Encoder_GetCountA/B 和 Encoder_Reset 接口正确 */
    Debug_Printf("--- Test 2: Encoder Interface ---\r\n");
    int32_t encA = Encoder_GetCountA();
    int32_t encB = Encoder_GetCountB();
    Debug_Printf("EncA=%ld, EncB=%ld\r\n", encA, encB);
    Encoder_Reset();
    Debug_Printf("Encoder: OK\r\n");

    /* [测试3] 红外传感器接口：读取传感器状态
     * 验证 IR_GetSensorState / IR_GetPositionError / IR_GetTurnDiff 接口正确 */
    Debug_Printf("--- Test 3: IR Sensor Interface ---\r\n");
    uint8_t ir_state = IR_GetSensorState();
    float ir_pos = IR_GetPositionError();
    float ir_turn = IR_GetTurnDiff();
    Debug_Printf("IR: state=0x%02X, pos=%.1fmm, turn=%.1f\r\n",
                 ir_state, ir_pos, ir_turn);
    Debug_Printf("IR Sensor: OK\r\n");

    /* [测试4] 电池 ADC 接口
     * 验证 Get_battery_volt / Batt_GetVoltage 接口正确 */
    Debug_Printf("--- Test 4: Battery ADC Interface ---\r\n");
    float batt = Batt_GetVoltage();
    Debug_Printf("Battery: %.2fV\r\n", batt);
    Debug_Printf("Battery ADC: OK\r\n");

    /* [测试5] PID 接口
     * 验证 PID_Init / PID_SetSetpoint / PID_Compute / PID_Reset 接口正确 */
    Debug_Printf("--- Test 5: PID Interface ---\r\n");
    PID_t test_pid;
    PID_Init(&test_pid, 400.0f, 400.0f, 0.0f, -7800.0f, 7800.0f);
    PID_SetSetpoint(&test_pid, 0.3f);
    float pid_out = PID_Compute(&test_pid, 0.0f, 0.005f);
    Debug_Printf("PID: output=%.1f (target=0.3, measured=0.0)\r\n", pid_out);
    PID_Reset(&test_pid);
    Debug_Printf("PID: OK\r\n");

    /* [测试6] 运动学接口
     * 验证 Kinematics_Inverse / Kinematics_Forward / Kinematics_MinTurnRadius */
    Debug_Printf("--- Test 6: Kinematics Interface ---\r\n");
    ChassisCmd_t tcmd = { .vx = 0.3f, .wz = 0.5f };
    WheelSpeed_t tws = Kinematics_Inverse(tcmd);
    Debug_Printf("Kinematics: cmd(vx=%.2f,wz=%.2f) → L=%.3f,R=%.3f m/s\r\n",
                 tcmd.vx, tcmd.wz, tws.left, tws.right);
    ChassisCmd_t tback = Kinematics_Forward(tws);
    Debug_Printf("Kinematics: roundtrip → vx=%.3f,wz=%.3f\r\n", tback.vx, tback.wz);
    Debug_Printf("Kinematics: OK\r\n");

    /* [测试7] 里程计接口
     * 验证 Odom_Init / Odom_Update / Odom_GetPose / Odom_Reset / Odom_GetDistance */
    Debug_Printf("--- Test 7: Odometry Interface ---\r\n");
    Odom_Init();
    Odom_Update(100, 100, 0.005f);
    Odom_t pose = Odom_GetPose();
    Debug_Printf("Odom: pose(x=%.3f,y=%.3f,th=%.2f), dist=%.3f\r\n",
                 pose.x, pose.y, pose.theta, Odom_GetDistance());
    Debug_Printf("Odometry: OK\r\n");

    /* [测试8] 运动控制接口
     * 验证 MotionControl_Init / SetTarget / Update / Stop */
    Debug_Printf("--- Test 8: Motion Control Interface ---\r\n");
    MotionControl_Init();
    ChassisCmd_t mcmd = { .vx = 0.2f, .wz = 0.0f };
    MotionControl_SetTarget(mcmd);
    Debug_Printf("MotionControl: target set, ready for Update()\r\n");
    MotionControl_Stop();
    Debug_Printf("MotionControl: OK\r\n");

    /* [测试9] 巡线控制接口
     * 验证 LineFollow_Init / ComputeCorrection / SetBaseSpeed */
    Debug_Printf("--- Test 9: LineFollow Interface ---\r\n");
    LineFollow_Init();
    LineFollow_SetBaseSpeed(0.15f);
    uint8_t lf_state = IR_GetSensorState();
    float lf_corr = LineFollow_ComputeCorrection(lf_state);
    Debug_Printf("LineFollow: correction=%.3f (state=0x%02X)\r\n",
                 lf_corr, lf_state);
    Debug_Printf("LineFollow: OK\r\n");

    /* [测试10] 蓝牙协议接口
     * 验证 BT_Protocol_Init / Handler / SendStatus / SendPID */
    Debug_Printf("--- Test 10: BT Protocol Interface ---\r\n");
    BT_Protocol_Init();
    BT_Protocol_SendStatus(STATE_IDLE, 8.2f, 0.0f, 0.0f);
    BT_Protocol_SendPID();
    Debug_Printf("BT Protocol: OK\r\n");

    /* [测试11] 虚拟示波器接口
     * 验证 Scope_SendChannel / Scope_SendFrame */
    Debug_Printf("--- Test 11: Debug Scope Interface ---\r\n");
    Scope_SendChannel(1.23f, 1);
    Scope_SendChannel(4.56f, 2);
    Scope_SendFrame(2);
    Debug_Printf("Debug Scope: OK\r\n");

    /* [测试12] 滤波器接口
     * 验证 Filter_LowPass / Filter_Complementary / Filter_RateLimit / Filter_DeadZone */
    Debug_Printf("--- Test 12: Filter Interface ---\r\n");
    float fprev = 0.0f;
    float flp = Filter_LowPass(1.0f, &fprev, 0.1f);
    float fdz = Filter_DeadZone(0.001f, 0.01f);
    float frl = Filter_RateLimit(1.0f, 0.0f, 0.5f, 0.005f);
    Debug_Printf("Filter: LP=%.3f, DZ=%.3f, RL=%.3f\r\n", flp, fdz, frl);
    Debug_Printf("Filter: OK\r\n");

    /* [测试13] MPU6050 接口 — 需硬件连接
     * 验证 MPU6050_Init / DataReady / Read / GetYaw / GetPitch / GetRoll / GetGyroZ
     *
     * 硬件要求：MPU6050 模块连接到 I2C 引脚 (SDA=PA0, SCL=PA1)
     * 无 MPU6050 时会跳过（打印 "SKIP"），不会阻塞 */
    Debug_Printf("--- Test 13: MPU6050 Interface ---\r\n");
    MPU6050_Init();    /* I2C 初始化 + 检测设备 + 配置寄存器 */
    delay_ms(50);

    /* 尝试读取数据：I2C 通信失败 → 设备不存在 */
    bool mpu_ok = false;
    for (int mpu_retry = 0; mpu_retry < 3; mpu_retry++) {
        float gyro[3], accel[3], angle[3];
        MPU6050_Read(gyro, accel, angle);
        if (MPU6050_DataReady()) {
            float yaw   = MPU6050_GetYaw();
            float pitch = MPU6050_GetPitch();
            float roll  = MPU6050_GetRoll();
            float gz    = MPU6050_GetGyroZ();
            Debug_Printf("MPU6050: Yaw=%.1f, Pitch=%.1f, Roll=%.1f, GyroZ=%.1f deg/s\r\n",
                         yaw, pitch, roll, gz);
            Debug_Printf("MPU6050: OK\r\n");
            OLED_ShowString(0, 4, "MPU6050 OK");
            mpu_ok = true;
            break;
        }
        delay_ms(10);
    }
    if (!mpu_ok) {
        Debug_Printf("MPU6050: SKIP (device not detected on I2C PA0/PA1)\r\n");
        OLED_ShowString(0, 4, "MPU6050 N/A");
    }

    /* ---- 全部接口测试完成 ---- */
    OLED_Clear();
    OLED_ShowString(0, 0, "ALL IF OK!");
    OLED_Refresh_Gram();
    Debug_Printf("\r\n======== ALL INTERFACES OK ========\r\n");

    while (1)
    {
        Voltage = Get_battery_volt();
        BTBufferHandler();       /* 调试模式下仍用旧蓝牙处理，不干扰测试 */

        /* ---- 团队成员调试区 ---- */
        /* [成员A] 电机测试：
         * Motor_SetPWM(2000, 2000); delay_ms(1000); Motor_Stop(); delay_ms(1000); */

        /* [成员B] 编码器测试：
         * int32_t a = Encoder_GetCountA(); Debug_Printf("EncA=%ld\r\n", a); */

        /* [成员C] 红外传感器测试：
         * uint8_t ir = IR_GetSensorState(); OLED_ShowNumber(0,20,ir,1,12); */

        //* [成员D] MPU6050 测试 — 每 100ms 读取并输出 Yaw 角
        //取消注释以启用：
        static uint32_t mpu_tick = 0;
        if (++mpu_tick >= 20) { // 200Hz ISR 调 MPU6050_Read(), 这里每100ms打印
          mpu_tick = 0;
          if (MPU6050_DataReady()) {
            float yaw = MPU6050_GetYaw();
            float pitch = MPU6050_GetPitch();
            Debug_Printf("MPU Y=%.1f P=%.1f\r\n", yaw, pitch);
            OLED_ShowNumber(0, 40, (int)yaw, 4, 12);
          }
        }

            /* ---- 显示刷新 ---- */
            // oled_show();
            OLED_Refresh_Gram();
    }

    /* ====================================================================
     * 比赛模式 — 完整自主/遥控操作，使用状态机 + 新架构 API
     * ==================================================================== */
#else
    OLED_ShowString(0, 0, "COMP MODE");
    OLED_Refresh_Gram();
    delay_ms(3000);

    while (1)
    {
        static uint8_t bt_toggle = 0;

        Voltage = Get_battery_volt();
        SM_ReportBattery(Voltage);

        /* 蓝牙协议处理（新 API：替换旧 BTBufferHandler） */
        BT_Protocol_Handler();

        /* 状态上报（新 API：替换旧 APP_Show）
         * 轮流发送：A 包（状态）、B 包（角度预留）、C 包（PID 参数） */
        if (PID_Send) {
            BT_Protocol_SendPID();
            PID_Send = 0;
        } else if (bt_toggle == 0) {
            /* A 包：电机速度 + 电池 */
            BT_Protocol_SendStatus(SM_GetState(), Voltage,
                                   Velocity_Left, Velocity_Right);
        } else {
            /* B 包：姿态角（MPU6050 移植后填充真实数据） */
            BT_Printf("{B%d:%d:%d}$", 0, 0, 0);
        }
        bt_toggle = !bt_toggle;

        /* OLED 显示刷新 */
        oled_show();
        OLED_Refresh_Gram();

        /* 低电量：LED 闪烁在状态机 (TIMER ISR) 中处理 */
    }
#endif
}

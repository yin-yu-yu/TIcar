/**
 * @file    debugmode.c
 * @brief   Debug mode — hardware interface verification and team test area
 *
 * OWNER:   All team members (add your test code in the designated areas)
 * STATUS:  Active — edit freely, empty.c is not affected
 *
 * ## How to use
 *
 * 1. Uncomment `#define CHASSIS_DEBUG` in empty.c
 * 2. Build and flash
 * 3. The 13 interface tests run once on boot, then the debug loop spins
 * 4. Add your test code in the team member areas of the debug loop
 *
 * ## Adding a new interface test
 *
 * Copy the pattern from an existing test:
 *   Debug_Printf("--- Test N: Your Module Interface ---\\r\\n");
 *   // ... initialize, exercise, verify ...
 *   Debug_Printf("Your Module: OK\\r\\n");
 *
 * Tests should be non-blocking if the hardware may be absent (see Test 13).
 */

#include "debugmode.h"
#include "board.h"
#include "show.h"
#include "uart_callback.h"

/* ---- External globals referenced in debug loop ---- */
extern float Voltage;
extern volatile bool g_DebugMode;
extern volatile bool g_ModeSwitchRequest;

/* ========================================================================
 * Static helpers — one per interface test
 * ======================================================================== */

static void Test_Motor(void)
{
    MotionControl_Init();
    Debug_Printf("--- Test 1: Motor Interface ---\r\n");
    Debug_Printf("Motor: Forward\r\n");
    Motor_SetPWM(8000, 8000);
    delay_ms(1500);
    Debug_Printf("Motor: Stop\r\n");
    Motor_Stop();
    delay_ms(1500);
    Debug_Printf("Motor: Backward\r\n");
    Motor_SetPWM(-8000, -8000);
    delay_ms(1500);
    Motor_Stop();
    Debug_Printf("Motor: OK\r\n");
}

static void Test_Encoder(void)
{
    Debug_Printf("--- Test 2: Encoder Interface ---\r\n");
    int32_t encA = Encoder_GetCountA();
    int32_t encB = Encoder_GetCountB();
    Debug_Printf("EncA=%ld, EncB=%ld\r\n", encA, encB);
    Encoder_Reset();
    Debug_Printf("Encoder: OK\r\n");
}

static void Test_IR(void)
{
    Debug_Printf("--- Test 3: IR Sensor Interface ---\r\n");
    uint8_t ir_state = IR_GetSensorState();
    float ir_pos = IR_GetPositionError();
    float ir_turn = IR_GetTurnDiff();
    Debug_Printf("IR: state=0x%02X, pos=%.1fmm, turn=%.1f\r\n",
                 ir_state, ir_pos, ir_turn);
    Debug_Printf("IR Sensor: OK\r\n");
}

static void Test_Battery(void)
{
    Debug_Printf("--- Test 4: Battery ADC Interface ---\r\n");
    float batt = Batt_GetVoltage();
    Debug_Printf("Battery: %.2fV\r\n", batt);
    Debug_Printf("Battery ADC: OK\r\n");
}

static void Test_PID(void)
{
    Debug_Printf("--- Test 5: PID Interface ---\r\n");
    PID_t test_pid;
    PID_Init(&test_pid, 400.0f, 400.0f, 0.0f, -7800.0f, 7800.0f);
    PID_SetSetpoint(&test_pid, 0.3f);
    float pid_out = PID_Compute(&test_pid, 0.0f, 0.005f);
    Debug_Printf("PID: output=%.1f (target=0.3, measured=0.0)\r\n", pid_out);
    PID_Reset(&test_pid);
    Debug_Printf("PID: OK\r\n");
}

static void Test_Kinematics(void)
{
    Debug_Printf("--- Test 6: Kinematics Interface ---\r\n");
    ChassisCmd_t tcmd = { .vx = 0.3f, .wz = 0.5f };
    WheelSpeed_t tws = Kinematics_Inverse(tcmd);
    Debug_Printf("Kinematics: cmd(vx=%.2f,wz=%.2f) -> L=%.3f,R=%.3f m/s\r\n",
                 tcmd.vx, tcmd.wz, tws.left, tws.right);
    ChassisCmd_t tback = Kinematics_Forward(tws);
    Debug_Printf("Kinematics: roundtrip -> vx=%.3f,wz=%.3f\r\n", tback.vx, tback.wz);
    Debug_Printf("Kinematics: OK\r\n");
}

static void Test_Odometry(void)
{
    Debug_Printf("--- Test 7: Odometry Interface ---\r\n");
    Odom_Init();
    Odom_Update(100, 100, 0.005f);
    Odom_t pose = Odom_GetPose();
    Debug_Printf("Odom: pose(x=%.3f,y=%.3f,th=%.2f), dist=%.3f\r\n",
                 pose.x, pose.y, pose.theta, Odom_GetDistance());
    Debug_Printf("Odometry: OK\r\n");
}

static void Test_MotionControl(void)
{
    Debug_Printf("--- Test 8: Motion Control Interface ---\r\n");
    MotionControl_Init();
    ChassisCmd_t mcmd = { .vx = 0.2f, .wz = 0.0f };
    MotionControl_SetTarget(mcmd);
    Debug_Printf("MotionControl: target set, ready for Update()\r\n");
    MotionControl_Stop();
    Debug_Printf("MotionControl: OK\r\n");
}

static void Test_LineFollow(void)
{
    Debug_Printf("--- Test 9: LineFollow Interface ---\r\n");
    LineFollow_Init();
    LineFollow_SetBaseSpeed(0.15f);
    uint8_t lf_state = IR_GetSensorState();
    float lf_corr = LineFollow_ComputeCorrection(lf_state);
    Debug_Printf("LineFollow: correction=%.3f (state=0x%02X)\r\n",
                 lf_corr, lf_state);
    Debug_Printf("LineFollow: OK\r\n");
}

static void Test_BTProtocol(void)
{
    Debug_Printf("--- Test 10: BT Protocol Interface ---\r\n");
    BT_Protocol_Init();
    BT_Protocol_SendStatus(STATE_IDLE, 8.2f, 0.0f, 0.0f);
    BT_Protocol_SendPID();
    Debug_Printf("BT Protocol: OK\r\n");
}

static void Test_Scope(void)
{
    Debug_Printf("--- Test 11: Debug Scope Interface ---\r\n");
    Scope_SendChannel(1.23f, 1);
    Scope_SendChannel(4.56f, 2);
    Scope_SendFrame(2);
    Debug_Printf("Debug Scope: OK\r\n");
}

static void Test_Filter(void)
{
    Debug_Printf("--- Test 12: Filter Interface ---\r\n");
    float fprev = 0.0f;
    float flp = Filter_LowPass(1.0f, &fprev, 0.1f);
    float fdz = Filter_DeadZone(0.001f, 0.01f);
    float frl = Filter_RateLimit(1.0f, 0.0f, 0.5f, 0.005f);
    Debug_Printf("Filter: LP=%.3f, DZ=%.3f, RL=%.3f\r\n", flp, fdz, frl);
    Debug_Printf("Filter: OK\r\n");
}

/**
 * @brief  Test 13: MPU6050 — skips gracefully if hardware not present
 *
 * Hardware required: MPU6050 module connected to I2C pins (SDA=PA0, SCL=PA1).
 * Without MPU6050 the test prints "SKIP" and continues — no blocking.
 */
static void Test_MPU6050(void)
{
    Debug_Printf("--- Test 13: MPU6050 Interface ---\r\n");
    MPU6050_Init();    /* I2C init + device detection + register config */
    delay_ms(50);

    /* Try to read data: I2C comm failure -> device absent */
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
}

/* ========================================================================
 * Public function
 * ======================================================================== */

void DebugMode_Run(void)
{
    static bool first_run = true;

    /* ---- Only run tests on first entry after reset ---- */
    if (first_run) {
        
        /* ---- Debug-mode boot banner ---- */
        OLED_ShowString(0, 0, "DEBUG MODE");
        OLED_Refresh_Gram();
        delay_ms(3000);

        /* ---- Interface test banner ---- */
        OLED_Clear();
        OLED_ShowString(0, 0, "IF TEST...");
        OLED_Refresh_Gram();

        /* ---- Run all interface verification tests ---- */
        Test_Motor();
        Test_Encoder();
        Test_IR();
        Test_Battery();
        Test_PID();
        Test_Kinematics();
        Test_Odometry();
        Test_MotionControl();
        Test_LineFollow();
        Test_BTProtocol();
        Test_Scope();
        Test_Filter();
        Test_MPU6050();

        /* ---- All tests complete ---- */
        OLED_Clear();
        OLED_ShowString(0, 0, "ALL IF OK!");
        OLED_Refresh_Gram();
        Debug_Printf("\r\n======== ALL INTERFACES OK ========\r\n");

        first_run = false;
    }

    /* ====================================================================
     * Debug main loop — team members add test code below
     * ==================================================================== */
    while (1)
    {
        /* ---- Mode switch: long-press key → competition mode ---- */
        if (g_ModeSwitchRequest) {
            g_ModeSwitchRequest = false;
            g_DebugMode = false;
            Motor_Stop();
            return;
        }

        Voltage = Get_battery_volt();
        BTBufferHandler();       /* Legacy BT handler — non-intrusive in debug mode */

        /* ---- Team member debug areas ---- */
        /* [成员A] 电机测试：
         * Motor_SetPWM(2000, 2000); delay_ms(1000); Motor_Stop(); delay_ms(1000); */

        /* [成员B] 编码器测试：
         * int32_t a = Encoder_GetCountA(); Debug_Printf("EncA=%ld\r\n", a); */

        /* [成员C] 红外传感器测试：
         * uint8_t ir = IR_GetSensorState(); OLED_ShowNumber(0,20,ir,1,12); */

        //* [成员D] MPU6050 测试 — 每 100ms 读取并输出 Yaw 角
        //取消注释以启用：
        static uint32_t mpu_tick = 0;
        if (++mpu_tick >= 20) { // 200Hz ISR calls MPU6050_Read(), print every 100ms here
          mpu_tick = 0;
          if (MPU6050_DataReady()) {
            float yaw = MPU6050_GetYaw();
            float pitch = MPU6050_GetPitch();
            Debug_Printf("MPU Y=%.1f P=%.1f\r\n", yaw, pitch);
            OLED_ShowNumber(0, 40, (int)yaw, 4, 12);
          }
        }

        /* ---- Display refresh ---- */
        // oled_show();
        OLED_Refresh_Gram();
    }
}

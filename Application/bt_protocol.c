/**
 * @file    bt_protocol.c
 * @brief   蓝牙协议解析器 — 兼容 WheelTec APP
 *
 * 负责人：团队成员
 * 状态：已完成 — 从 Control/uart_callback.c + Control/show.c 迁移
 *
 * 协议：WheelTec 二进制命令格式
 *   - 方向：  0x41~0x48 → 8 个摇杆方向
 *   - 转向：  0x4B → 切换转向模式；0x43→右转, 0x47→左转
 *   - 速度：  0x58 → +100mm/s;      0x59 → -100mm/s
 *   - PID 设置：{0x7B, 0x23, ParamID, Data..., 0x7D}
 *   - PID 查询：ParamID=0x50 → 置位 PID_Send 标志
 *   - 状态：  "{A%d:%d:%d:%d}$" (speedL, speedR, batt%, 0)
 *   - PID 上报："{C%d:%d:%d:%d:%d:%d:%d:%d:%d:$}" (所有参数)
 *
 * 硬件：UART1, DMA CH0, PB6(TX)/PB7(RX), 9600bps
 */

#include "bt_protocol.h"
#include "uart_bt.h"
#include "robot_config.h"
#include "pid_config.h"
#include "board.h"
#include <string.h>
#include <math.h>

/* ========================================================================
 * 数据包缓冲区（与旧版 BT_PACKET_SIZE 保持一致）
 * ======================================================================== */
#define BT_PACKET_SIZE  200

static volatile uint8_t g_rx_buffer[BT_PACKET_SIZE];

/* ========================================================================
 * 外部标志 — 由本模块设置，state_machine 和 control 读取
 * ======================================================================== */
extern int  Flag_Left, Flag_Right, Flag_Direction, Turn_Flag;
extern uint8_t Flag_Stop;
extern int  Run_Mode;

extern float RC_Velocity;
extern float Velocity_KP, Velocity_KI;

/* 循线参数（可通过蓝牙 APP 调节） */
extern float Turn90Angle;
extern float TurnMaxAngle;
extern float TurnMidAngle;
extern float TurnMinAngle;
extern float BaseSpeed;
extern float ForwardLimit;

/* ========================================================================
 * PID_Send 标志 — 在 board.h 中声明为 extern，由蓝牙命令 0x50 置位
 * ======================================================================== */
extern uint8_t PID_Send;

/* ========================================================================
 * 前向声明
 * ======================================================================== */
static void bt_control(uint8_t recv);

/* ========================================================================
 * 公开函数
 * ======================================================================== */

void BT_Protocol_Init(void)
{
    /* 为 UART1 接收配置 DMA。
     * UART1 已由 SysConfig 初始化。
     * 此处设置 DMA CH0 接收数据到 g_rx_buffer。 */
    BT_DMAConfig();
}

/**
 * @brief  处理蓝牙接收缓冲区（从主循环中调用）
 *
 * 从 uart_callback.c 的 BTBufferHandler() 移植。
 *
 * 策略：
 *   - 轮询 DMA 传输大小以检测新字节
 *   - 新数据到达时，记录时间戳并置位处理标志
 *   - 空闲 1ms 后，通过 bt_control() 处理所有新字节
 *   - 缓冲区满一半时重启 DMA（防止溢出）
 */
void BT_Protocol_Handler(void)
{
    static uint8_t  handle_flag  = 0;
    static uint8_t  handle_size  = 0;
    static uint8_t  last_size    = 0;
    static uint32_t last_tick    = 0;

    /* 已接收字节数 = 总缓冲区 - 剩余传输量 */
    uint8_t recvsize = (uint8_t)(BT_PACKET_SIZE
                      - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID));

    if (recvsize != last_size) {
        /* 新数据到达 — 启动处理器 */
        handle_flag = 1;
        last_tick   = Systick_getTick();
    } else {
        /* 无新数据 — 空闲 1ms 超时后处理 */
        if (handle_flag == 1
            && ((last_tick - Systick_getTick()) & SysTickMAX_COUNT) >= SysTick_MS(1))
        {
            handle_flag = 0;

            /* 通过命令解析器处理每个新字节 */
            for (uint8_t i = handle_size; i < recvsize; i++) {
                bt_control(g_rx_buffer[i]);
            }
            handle_size = recvsize;

            /* 防溢出：缓冲区 ≥ 半满时重启 DMA */
            if (recvsize >= BT_PACKET_SIZE / 2) {
                handle_size = 0;
                last_size   = 0;
                BT_DMAConfig();
            }
        }
    }

    last_size = recvsize;
}

void BT_Protocol_SendStatus(RobotState_t state, float batt_v,
                            float speed_l, float speed_r)
{
    /* 电池电压 → 百分比（2S LiPo：6.5V=0%, 8.4V=100%） */
    int batt_pct = (int)((batt_v - BATT_CRITICAL_THRESHOLD_V)
                   / (BATT_FULL_V - BATT_CRITICAL_THRESHOLD_V) * 100.0f);
    if (batt_pct < 0)   batt_pct = 0;
    if (batt_pct > 100) batt_pct = 100;

    /* 速度转换 m/s → mm/s（APP 期望 mm/s） */
    int sl = (int)fabsf(speed_l * 1.1f * 1000.0f);
    int sr = (int)fabsf(speed_r * 1.1f * 1000.0f);

    /* 格式：{A_左速度:右速度:电量百分比:保留}$ */
    BT_Printf("{A%d:%d:%d:%d}$", sl, sr, batt_pct, 0);
    (void)state;  /* 保留供将来使用 */
}

void BT_Protocol_SendPID(void)
{
    /* 格式：{C_KP:KI:基础速度:T90:TMax:TMid:TMin:前向限幅:保留:$} */
    BT_Printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d:$}",
              (int)Velocity_KP, (int)Velocity_KI,
              (int)BaseSpeed,
              (int)Turn90Angle,  (int)TurnMaxAngle,
              (int)TurnMidAngle, (int)TurnMinAngle,
              (int)ForwardLimit, 0);
}

/* ========================================================================
 * 私有：命令解析器
 *
 * 从 uart_callback.c 的 bt_control() 移植。
 * 每个接收到的字节通过此状态机进行分发。
 * ======================================================================== */

static void bt_control(uint8_t recv)
{
    static int     Usart_Receive = 0;
    static uint8_t Flag_PID      = 0;
    static uint8_t i, j;
    static uint8_t Receive[50];
    static float   Data;

    Usart_Receive = recv;

    /* ---- 模式检测：转向 vs 方向 ---- */
    if (Usart_Receive == 0x4B) {
        /* 进入 APP 转向控制界面 */
        Turn_Flag = 1;
    } else if (Usart_Receive == 0x49 || Usart_Receive == 0x4A) {
        /* 进入 APP 方向控制界面 */
        Turn_Flag = 0;
    }

    /* ---- 方向控制模式 ---- */
    if (Turn_Flag == 0) {
        /* APP 摇杆：0x41~0x48 = 8 方向 (1~8) */
        if (Usart_Receive >= 0x41 && Usart_Receive <= 0x48) {
            Flag_Direction = Usart_Receive - 0x40;
        } else if (Usart_Receive <= 8) {
            Flag_Direction = Usart_Receive;
        } else {
            Flag_Direction = 0;
        }
    }
    /* ---- 转向控制模式 ---- */
    else if (Turn_Flag == 1) {
        if (Usart_Receive == 0x43) {
            /* 右转 */
            Flag_Left  = 0;
            Flag_Right = 1;
        } else if (Usart_Receive == 0x47) {
            /* 左转 */
            Flag_Left  = 1;
            Flag_Right = 0;
        } else {
            Flag_Left  = 0;
            Flag_Right = 0;
        }

        if (Usart_Receive == 0x41 || Usart_Receive == 0x45) {
            Flag_Direction = Usart_Receive - 0x40;
        } else {
            Flag_Direction = 0;
        }
    }

    /* ---- 速度调节 ---- */
    if (Usart_Receive == 0x58) {
        /* 加速：+100mm/s */
        RC_Velocity += 100.0f;
        if (RC_Velocity > (MAX_LINEAR_SPEED_MPS * 1000.0f)) {
            RC_Velocity = MAX_LINEAR_SPEED_MPS * 1000.0f;
        }
    }
    if (Usart_Receive == 0x59) {
        /* 减速：-100mm/s */
        RC_Velocity -= 100.0f;
        if (RC_Velocity < 100.0f) {
            RC_Velocity = 100.0f;  /* 最低速度 */
        }
    }

    /* ---- PID 参数设置帧：{0x7B, 0x23, ParamID, Data..., 0x7D} ---- */
    if (Usart_Receive == 0x7B) {
        Flag_PID = 1;  /* 帧起始 */
    }
    if (Usart_Receive == 0x7D) {
        Flag_PID = 2;  /* 帧结束 */
    }

    if (Flag_PID == 1) {
        /* 收集帧字节 */
        Receive[i] = Usart_Receive;
        i++;
    }

    if (Flag_PID == 2) {
        /* ---- 解析收集到的帧 ---- */
        if (Receive[3] == 0x50) {
            /* PID 查询请求 → 为下一状态周期置位标志 */
            PID_Send = 1;
        } else if (Receive[1] != 0x23) {
            /* 解码 ASCII 数字参数值 */
            Data = 0.0f;
            for (j = i; j >= 4; j--) {
                Data += (float)(Receive[j - 1] - '0') * powf(10.0f, (float)(i - j));
            }

            /* 按 ParamID 分发 */
            switch (Receive[1]) {
                case 0x30:  Velocity_KP  = Data;  break;
                case 0x31:  Velocity_KI  = Data;  break;
                case 0x32:  BaseSpeed    = Data;  break;
                case 0x33:  Turn90Angle  = Data;  break;
                case 0x34:  TurnMaxAngle = Data;  break;
                case 0x35:  TurnMidAngle = Data;  break;
                case 0x36:  TurnMinAngle = Data;  break;
                case 0x37:  ForwardLimit = Data;  break;
                case 0x38:  /* 保留 */              break;
                default:    /* 未知参数 */          break;
            }
        }

        /* 复位解析器状态 */
        Flag_PID = 0;
        i = 0;
        j = 0;
        Data = 0.0f;
        memset(Receive, 0, sizeof(Receive));
    }
}

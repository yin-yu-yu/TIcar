/**
 * @file    bt_protocol.c
 * @brief   Bluetooth protocol parser — WheelTec APP compatible
 *
 * OWNER:  Team Member
 * STATUS: COMPLETE — migrated from Control/uart_callback.c + Control/show.c
 *
 * Protocol: WheelTec binary command format
 *   - Direction: 0x41~0x48 → 8 joystick directions
 *   - Steering: 0x4B → toggle steer mode; 0x43→right, 0x47→left
 *   - Speed:     0x58 → +100mm/s;     0x59 → -100mm/s
 *   - PID set:   {0x7B, 0x23, ParamID, Data..., 0x7D}
 *   - PID query: ParamID=0x50 → set PID_Send flag
 *   - Status:    "{A%d:%d:%d:%d}$" (speedL, speedR, batt%, 0)
 *   - PID rpt:   "{C%d:%d:%d:%d:%d:%d:%d:%d:%d:$}" (all params)
 *
 * Hardware: UART1, DMA CH0, PB6(TX)/PB7(RX), 9600bps
 */

#include "bt_protocol.h"
#include "uart_bt.h"
#include "robot_config.h"
#include "pid_config.h"
#include "board.h"
#include <string.h>
#include <math.h>

/* ========================================================================
 * Packet Buffer (matches legacy BT_PACKET_SIZE)
 * ======================================================================== */
#define BT_PACKET_SIZE  200

static volatile uint8_t g_rx_buffer[BT_PACKET_SIZE];

/* ========================================================================
 * Extern flags — set by this module, read by state_machine & control
 * ======================================================================== */
extern int  Flag_Left, Flag_Right, Flag_Direction, Turn_Flag;
extern uint8_t Flag_Stop;
extern int  Run_Mode;

extern float RC_Velocity;
extern float Velocity_KP, Velocity_KI;

/* Line-follow parameters (tunable via BT APP) */
extern float Turn90Angle;
extern float TurnMaxAngle;
extern float TurnMidAngle;
extern float TurnMinAngle;
extern float BaseSpeed;
extern float ForwardLimit;

/* ========================================================================
 * PID_Send flag — extern in board.h, set by BT command 0x50
 * ======================================================================== */
extern uint8_t PID_Send;

/* ========================================================================
 * Forward Declarations
 * ======================================================================== */
static void bt_control(uint8_t recv);

/* ========================================================================
 * Public Functions
 * ======================================================================== */

void BT_Protocol_Init(void)
{
    /* Configure DMA for UART1 reception.
     * UART1 is already initialized by SysConfig.
     * This sets up DMA CH0 to receive into g_rx_buffer. */
    BT_DMAConfig();
}

/**
 * @brief  Process Bluetooth receive buffer (call from main loop)
 *
 * Ported from uart_callback.c BTBufferHandler().
 *
 * Strategy:
 *   - Poll DMA transfer size to detect new bytes
 *   - When new data arrives, stamp a tick and set handle flag
 *   - After 1ms idle, process all new bytes through bt_control()
 *   - Restart DMA when buffer is half full (overflow prevention)
 */
void BT_Protocol_Handler(void)
{
    static uint8_t  handle_flag  = 0;
    static uint8_t  handle_size  = 0;
    static uint8_t  last_size    = 0;
    static uint32_t last_tick    = 0;

    /* Bytes received so far = total_buffer - remaining_transfer */
    uint8_t recvsize = (uint8_t)(BT_PACKET_SIZE
                      - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID));

    if (recvsize != last_size) {
        /* New data arrived — arm the handler */
        handle_flag = 1;
        last_tick   = Systick_getTick();
    } else {
        /* No new data — process after 1ms idle timeout */
        if (handle_flag == 1
            && ((last_tick - Systick_getTick()) & SysTickMAX_COUNT) >= SysTick_MS(1))
        {
            handle_flag = 0;

            /* Process each new byte through the command parser */
            for (uint8_t i = handle_size; i < recvsize; i++) {
                bt_control(g_rx_buffer[i]);
            }
            handle_size = recvsize;

            /* Overflow prevention: restart DMA when buffer ≥ half full */
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
    /* Convert battery voltage → percentage (2S LiPo: 6.5V=0%, 8.4V=100%) */
    int batt_pct = (int)((batt_v - BATT_CRITICAL_THRESHOLD_V)
                   / (BATT_FULL_V - BATT_CRITICAL_THRESHOLD_V) * 100.0f);
    if (batt_pct < 0)   batt_pct = 0;
    if (batt_pct > 100) batt_pct = 100;

    /* Convert speed m/s → mm/s (APP expects mm/s) */
    int sl = (int)fabsf(speed_l * 1.1f * 1000.0f);
    int sr = (int)fabsf(speed_r * 1.1f * 1000.0f);

    /* Format: {A_leftSpeed:rightSpeed:battPercent:reserved}$ */
    BT_Printf("{A%d:%d:%d:%d}$", sl, sr, batt_pct, 0);
    (void)state;  /* Reserved for future use */
}

void BT_Protocol_SendPID(void)
{
    /* Format: {C_KP:KI:BaseSpeed:T90:TMax:TMid:TMin:FLimit:reserved:$} */
    BT_Printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d:$}",
              (int)Velocity_KP, (int)Velocity_KI,
              (int)BaseSpeed,
              (int)Turn90Angle,  (int)TurnMaxAngle,
              (int)TurnMidAngle, (int)TurnMinAngle,
              (int)ForwardLimit, 0);
}

/* ========================================================================
 * Private: Command Parser
 *
 * Ported from uart_callback.c bt_control().
 * Each received byte is dispatched through this state machine.
 * ======================================================================== */

static void bt_control(uint8_t recv)
{
    static int     Usart_Receive = 0;
    static uint8_t Flag_PID      = 0;
    static uint8_t i, j;
    static uint8_t Receive[50];
    static float   Data;

    Usart_Receive = recv;

    /* ---- Mode detection: steering vs direction ---- */
    if (Usart_Receive == 0x4B) {
        /* Enter APP steering control interface */
        Turn_Flag = 1;
    } else if (Usart_Receive == 0x49 || Usart_Receive == 0x4A) {
        /* Enter APP direction control interface */
        Turn_Flag = 0;
    }

    /* ---- Direction control mode ---- */
    if (Turn_Flag == 0) {
        /* APP joystick: 0x41~0x48 = 8 directions (1~8) */
        if (Usart_Receive >= 0x41 && Usart_Receive <= 0x48) {
            Flag_Direction = Usart_Receive - 0x40;
        } else if (Usart_Receive <= 8) {
            Flag_Direction = Usart_Receive;
        } else {
            Flag_Direction = 0;
        }
    }
    /* ---- Steering control mode ---- */
    else if (Turn_Flag == 1) {
        if (Usart_Receive == 0x43) {
            /* Right rotation */
            Flag_Left  = 0;
            Flag_Right = 1;
        } else if (Usart_Receive == 0x47) {
            /* Left rotation */
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

    /* ---- Speed adjustment ---- */
    if (Usart_Receive == 0x58) {
        /* Accelerate: +100mm/s */
        RC_Velocity += 100.0f;
        if (RC_Velocity > (MAX_LINEAR_SPEED_MPS * 1000.0f)) {
            RC_Velocity = MAX_LINEAR_SPEED_MPS * 1000.0f;
        }
    }
    if (Usart_Receive == 0x59) {
        /* Decelerate: -100mm/s */
        RC_Velocity -= 100.0f;
        if (RC_Velocity < 100.0f) {
            RC_Velocity = 100.0f;  /* Minimum speed */
        }
    }

    /* ---- PID parameter setting frame: {0x7B, 0x23, ParamID, Data..., 0x7D} ---- */
    if (Usart_Receive == 0x7B) {
        Flag_PID = 1;  /* Frame start */
    }
    if (Usart_Receive == 0x7D) {
        Flag_PID = 2;  /* Frame end */
    }

    if (Flag_PID == 1) {
        /* Collect frame bytes */
        Receive[i] = Usart_Receive;
        i++;
    }

    if (Flag_PID == 2) {
        /* ---- Parse collected frame ---- */
        if (Receive[3] == 0x50) {
            /* PID query request → set flag for next status cycle */
            PID_Send = 1;
        } else if (Receive[1] != 0x23) {
            /* Decode ASCII numeric parameter value */
            Data = 0.0f;
            for (j = i; j >= 4; j--) {
                Data += (float)(Receive[j - 1] - '0') * powf(10.0f, (float)(i - j));
            }

            /* Dispatch by ParamID */
            switch (Receive[1]) {
                case 0x30:  Velocity_KP  = Data;  break;
                case 0x31:  Velocity_KI  = Data;  break;
                case 0x32:  BaseSpeed    = Data;  break;
                case 0x33:  Turn90Angle  = Data;  break;
                case 0x34:  TurnMaxAngle = Data;  break;
                case 0x35:  TurnMidAngle = Data;  break;
                case 0x36:  TurnMinAngle = Data;  break;
                case 0x37:  ForwardLimit = Data;  break;
                case 0x38:  /* Reserved */        break;
                default:    /* Unknown param */    break;
            }
        }

        /* Reset parser state */
        Flag_PID = 0;
        i = 0;
        j = 0;
        Data = 0.0f;
        memset(Receive, 0, sizeof(Receive));
    }
}

#include "IR_Module.h"
#include "control.h"
uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;
/*=============================================================================
 * 转弯角度参数
 *=============================================================================*/
// 转弯角度参数
float Turn90Angle  = 70;   // 直角弯转弯角度
float TurnMaxAngle = 45;   // 大转弯角度
float TurnMidAngle = 20;   // 中等转弯角度（丢失时默认转弯角度）
float TurnMinAngle = 15;   // 微调转弯角度
extern int temp;
// 基础速度参数
float BaseSpeed = 150;      // 基础巡航速度，直线时的速度
float ForwardLimit = 70;		//前向限幅(转弯时束缚前进速度的下限比例)
/*=============================================================================
 * 传感器状态枚举--识别到对应位时为1
 *=============================================================================*/
typedef enum {
    STATE_CROSS         = 0,    // 0000 - 十字路口（全白）
    STATE_LEFT_90_A     = 1,    // 0001 - 左直角弯
	STATE_LEFT_90_B		= 3,	// 0011
    STATE_RIGHT_90_A    = 8,  	// 1000 - 右直角弯
	STATE_RIGHT_90_B    = 12,	// 1100
    STATE_LEFT_BIG      = 7,    // 0111 - 大左转
    STATE_RIGHT_BIG     = 14,   // 1110 - 大右转
    STATE_LEFT_SMALL    = 11,   // 1011 - 小左转
    STATE_RIGHT_SMALL   = 13,   // 1101 - 小右转
    STATE_STRAIGHT      = 9,    // 1001 - 直行
    STATE_LOST          = 15    // 1111 - 丢失（全黑）
} SensorState_t;
float base_speed_mm = 0;        // 基础速度，mm/s
float turn_diff = 0;            // 转弯差速
/*=============================================================================
 * 巡线核心函数：根据传感器状态计算左右轮目标速度
 *=============================================================================*/
void IRDM_line_inspection(void)
{
    static int last_state = 0;      // 记录上一次的状态
	float left_motor_speed = 0;     // 左轮实时速度（m/s）
    float right_motor_speed = 0;    // 右轮实时速度（m/s）
	static int turn_cnt=0;
	static int saved_state = 0;  // 保存转弯状态
    // 读取红外传感器状态（4个红外对管数字值）
	    // 读取各红外对管的状态，强制转换为0或1
    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0;
    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0;
    ir_dh2_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0;
    ir_dh1_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0;

    int sensor_state = (ir_dh1_state << 3) | (ir_dh2_state << 2) | (ir_dh3_state << 1) | ir_dh4_state; // 将四位传感器状态合成一个整数

    // 直角转弯延时处理：检测到直角后先直行200拍再转弯
    if((sensor_state == STATE_LEFT_90_A || sensor_state == STATE_RIGHT_90_A||sensor_state == STATE_LEFT_90_B || sensor_state == STATE_RIGHT_90_B) && turn_cnt == 0)
    {
        saved_state = sensor_state;  // 记住转弯状态
        turn_cnt = 1;
    }
    if(turn_cnt > 0)
    {
        if(turn_cnt < 175) sensor_state = STATE_STRAIGHT;  // 前200拍直行
        else if(turn_cnt < 4000&&sensor_state!=STATE_LEFT_BIG&&sensor_state!=STATE_RIGHT_BIG) sensor_state = saved_state;
        else { turn_cnt = 0; saved_state = 0; }
        if(turn_cnt > 0) turn_cnt++;
    }
  /*=========================================================================*
     * 状态判断，决定转弯方向和差速                                                *
     *=========================================================================*/
    switch (sensor_state)
    {
        case STATE_CROSS:// 十字路口（全白）
			turn_diff = 0;
            break;
        case STATE_LEFT_90_A: // 左直角弯
		case STATE_LEFT_90_B: // 左直角弯
            turn_diff = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // 右直角弯
		case STATE_RIGHT_90_B: // 右直角弯
            turn_diff = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://大左转
            turn_diff = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG://大右转
            turn_diff = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL://小左转
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL://小右转
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT://直行
            turn_diff = 0;
            break;
        case STATE_LOST://丢失（全黑）
            if (last_state == STATE_LEFT_SMALL) turn_diff = TurnMidAngle;//丢失后左转
			else if (last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;//丢失后右转
			else if(last_state == STATE_LEFT_BIG ) turn_diff = TurnMaxAngle;//丢失后左转
			else if(last_state == STATE_RIGHT_BIG ) turn_diff = -TurnMaxAngle;//丢失后右转
            break;
        default: // 未识别状态，直行
            turn_diff = 0;
            break;
    }
	//更新上一次状态
	if(sensor_state!=STATE_LOST)
	{
		last_state=sensor_state;
	}
    // 转弯角度越大，基础速度越小
	if(fabs(turn_diff)<ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
	}
	else base_speed_mm=0;
    /*========================================================================*
     * 计算左右轮速度：（基础速度-转弯差速，基础速度+转弯差速）单位 mm/s          *
     *=========================================================================*/
	left_motor_speed = 0.001f * (base_speed_mm - turn_diff);
    right_motor_speed = 0.001f * (base_speed_mm + turn_diff);
    // 赋值给左右轮速度
    MotorA.Target_Encoder = left_motor_speed;//左轮
    MotorB.Target_Encoder = right_motor_speed;//右轮
}

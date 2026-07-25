#include "IR_Module.h"
#include "control.h"
uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;
/*=============================================================================
 * 锟缴碉拷锟斤拷锟斤拷锟斤拷锟斤拷
 *=============================================================================*/
// 转锟斤拷嵌炔锟斤拷锟�
float Turn90Angle  = 70;   // 直锟斤拷锟斤拷转锟斤拷锟斤拷锟�
float TurnMaxAngle = 45;   // 锟斤拷锟斤拷锟阶�锟斤拷锟斤拷锟�
float TurnMidAngle = 20;   // 锟叫碉拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟绞笔癸拷茫锟�
float TurnMinAngle = 15;   // 微锟斤拷转锟斤拷锟斤拷锟�
extern int temp;
// 锟劫度诧拷锟斤拷
float BaseSpeed = 150;      // 锟斤拷锟斤拷巡锟斤拷锟劫度ｏ拷直锟斤拷时锟斤拷锟劫度ｏ拷
float ForwardLimit = 70;		//前锟斤拷锟斤拷锟斤拷(转锟斤拷锟斤拷诟锟街碉拷锟斤拷锟斤拷锟角帮拷锟�)
/*=============================================================================
 * 锟斤拷锟斤拷锟斤拷状态锟斤拷锟斤拷--识锟金到猴拷锟斤拷时为1
 *=============================================================================*/
typedef enum {
    STATE_CROSS         = 0,    // 0000 - 十锟斤拷路锟节ｏ拷全锟节ｏ拷
    STATE_LEFT_90_A     = 1,    // 0001 - 锟斤拷直锟斤拷锟斤拷
	STATE_LEFT_90_B		= 3,	// 0011
    STATE_RIGHT_90_A    = 8,  	// 1000 - 锟斤拷直锟斤拷锟斤拷
	STATE_RIGHT_90_B    = 12,	// 1100
    STATE_LEFT_BIG      = 7,    // 0111 - 锟斤拷锟斤拷锟�
    STATE_RIGHT_BIG     = 14,   // 1110 - 锟揭达拷锟斤拷
    STATE_LEFT_SMALL    = 11,   // 1011 - 锟斤拷微锟斤拷
    STATE_RIGHT_SMALL   = 13,   // 1101 - 锟斤拷微锟斤拷
    STATE_STRAIGHT      = 9,    // 1001 - 直锟斤拷
    STATE_LOST          = 15    // 1111 - 锟斤拷锟竭ｏ拷全锟阶ｏ拷
} SensorState_t;
float base_speed_mm = 0;        // 锟斤拷锟斤拷锟劫度ｏ拷mm/s锟斤拷
float turn_diff = 0;            // 转锟斤拷锟斤拷锟�
/*=============================================================================
 * 巡锟竭癸拷锟杰猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷目锟斤拷锟劫度ｏ拷
 *=============================================================================*/
void IRDM_line_inspection(void)
{
    static int last_state = 0;      // 锟斤拷录锟斤拷一锟轿碉拷状态
	float left_motor_speed = 0;     // 锟斤拷锟斤拷锟斤拷时锟劫度ｏ拷m/s锟斤拷
    float right_motor_speed = 0;    // 锟揭碉拷锟斤拷锟绞憋拷俣龋锟絤/s锟斤拷
	static int turn_cnt=0;
	static int saved_state = 0;  // 锟斤拷锟斤拷转锟斤拷状态
    // 锟斤拷取锟斤拷锟斤拷锟斤拷状态锟斤拷4锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟街�
	    // 锟斤拷取锟侥革拷锟斤拷锟脚碉拷状态锟斤拷强锟斤拷转锟斤拷为0锟斤拷1
    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0;
    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0;
    ir_dh2_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0;
    ir_dh1_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0;
	
    int sensor_state = (ir_dh1_state << 3) | (ir_dh2_state << 2) | (ir_dh3_state << 1) | ir_dh4_state; // 锟斤拷锟斤拷锟斤拷锟斤拷状态锟斤拷铣锟揭伙拷锟斤拷锟斤拷锟�
	
    // 直锟斤拷锟斤拷锟斤拷锟斤拷式锟斤拷锟斤拷锟斤拷前200锟斤拷直锟叫ｏ拷锟斤拷转锟斤拷
    if((sensor_state == STATE_LEFT_90_A || sensor_state == STATE_RIGHT_90_A||sensor_state == STATE_LEFT_90_B || sensor_state == STATE_RIGHT_90_B) && turn_cnt == 0)
    {
        saved_state = sensor_state;  // 锟斤拷住转锟斤拷状态
        turn_cnt = 1;
    }
    if(turn_cnt > 0)
    {
        if(turn_cnt < 175) sensor_state = STATE_STRAIGHT;  // 前200锟斤拷直锟斤拷
        else if(turn_cnt < 4000&&sensor_state!=STATE_LEFT_BIG&&sensor_state!=STATE_RIGHT_BIG) sensor_state = saved_state; 
        else { turn_cnt = 0; saved_state = 0; }  
        if(turn_cnt > 0) turn_cnt++; 
    }
  /*=========================================================================*
     * 状态锟叫断ｏ拷锟斤拷锟斤拷转锟斤拷锟斤拷锟�												   *
     *=========================================================================*/
    switch (sensor_state)
    {
        case STATE_CROSS:// 锟斤拷锟斤拷路锟节达拷锟斤拷
			turn_diff = 0;
            break;
        case STATE_LEFT_90_A: // 锟斤拷直锟斤拷锟斤拷
		case STATE_LEFT_90_B: // 锟斤拷直锟斤拷锟斤拷
            turn_diff = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // 锟斤拷直锟斤拷锟斤拷
		case STATE_RIGHT_90_B: // 锟斤拷直锟斤拷锟斤拷
            turn_diff = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://锟斤拷锟斤拷锟�
            turn_diff = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG://锟揭达拷锟斤拷
            turn_diff = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL://锟斤拷微锟斤拷
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL://锟斤拷微锟斤拷
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT://直锟斤拷
            turn_diff = 0;
            break;
        case STATE_LOST://锟斤拷锟竭达拷锟斤拷
            if (last_state == STATE_LEFT_SMALL) turn_diff = TurnMidAngle;//锟斤拷锟斤拷锟斤拷转
			else if (last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;//锟斤拷锟斤拷锟斤拷转
			else if(last_state == STATE_LEFT_BIG ) turn_diff = TurnMaxAngle;//锟斤拷锟斤拷锟斤拷转
			else if(last_state == STATE_RIGHT_BIG ) turn_diff = -TurnMaxAngle;//锟斤拷锟斤拷锟斤拷转
            break;
        default: // 未锟斤拷锟斤拷状态锟斤拷直锟斤拷
            turn_diff = 0;
            break;
    }
	//锟斤拷锟芥传锟斤拷锟斤拷状态
	if(sensor_state!=STATE_LOST)
	{
		last_state=sensor_state;
	}
    // 转锟斤拷锟劫讹拷越锟襟，伙拷锟斤拷锟劫讹拷越锟斤拷
	if(fabs(turn_diff)<ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
	}
	else base_speed_mm=0;
    /*========================================================================*
     * 锟斤拷锟矫碉拷锟侥匡拷锟斤拷俣龋锟斤拷锟�-转锟斤拷锟斤拷伲锟斤拷锟�+转锟斤拷锟斤拷伲锟斤拷锟轿伙拷锟絤m/s锟斤拷                   *
     *=========================================================================*/
	left_motor_speed = 0.001f * (base_speed_mm - turn_diff); 
    right_motor_speed = 0.001f * (base_speed_mm + turn_diff);
    // 锟斤拷值锟斤拷锟斤拷锟侥匡拷锟斤拷俣锟�
    MotorA.Target_Encoder = left_motor_speed;//锟斤拷锟斤拷
    MotorB.Target_Encoder = right_motor_speed;//锟揭碉拷锟�
}









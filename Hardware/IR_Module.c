#include "IR_Module.h"
#include "control.h"
uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;
/*=============================================================================
 * �ɵ���������
 *=============================================================================*/
// ת��ǶȲ���
float Turn90Angle  = 70;   // ֱ����ת�����
float TurnMaxAngle = 45;   // �����ת�����
float TurnMidAngle = 20;   // �е�ת�����������ʱʹ�ã�
float TurnMinAngle = 15;   // ΢��ת�����
extern int temp;
// �ٶȲ���
float BaseSpeed = 150;      // ����Ѳ���ٶȣ�ֱ��ʱ���ٶȣ�
float ForwardLimit = 70;		//ǰ������(ת����ڸ�ֵ������ǰ��)
/*=============================================================================
 * ������״̬����--ʶ�𵽺���ʱΪ1
 *=============================================================================*/
typedef enum {
    STATE_CROSS         = 0,    // 0000 - ʮ��·�ڣ�ȫ�ڣ�
    STATE_LEFT_90_A     = 1,    // 0001 - ��ֱ����
	STATE_LEFT_90_B		= 3,	// 0011
    STATE_RIGHT_90_A    = 8,  	// 1000 - ��ֱ����
	STATE_RIGHT_90_B    = 12,	// 1100
    STATE_LEFT_BIG      = 7,    // 0111 - �����
    STATE_RIGHT_BIG     = 14,   // 1110 - �Ҵ���
    STATE_LEFT_SMALL    = 11,   // 1011 - ��΢��
    STATE_RIGHT_SMALL   = 13,   // 1101 - ��΢��
    STATE_STRAIGHT      = 9,    // 1001 - ֱ��
    STATE_LOST          = 15    // 1111 - ���ߣ�ȫ�ף�
} SensorState_t;
float base_speed_mm = 0;        // �����ٶȣ�mm/s��
float turn_diff = 0;            // ת�����
/*=============================================================================
 * Ѳ�߹��ܺ�������������Ŀ���ٶȣ�
 *=============================================================================*/
void IRDM_line_inspection(void)
{
    static int last_state = 0;      // ��¼��һ�ε�״̬
	float left_motor_speed = 0;     // ������ʱ�ٶȣ�m/s��
    float right_motor_speed = 0;    // �ҵ����ʱ�ٶȣ�m/s��
	static int turn_cnt=0;
	static int saved_state = 0;  // ����ת��״̬
    // ��ȡ������״̬��4�����������ֵ
	    // ��ȡ�ĸ����ŵ�״̬��ǿ��ת��Ϊ0��1
    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0;
    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0;
    ir_dh2_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0;
    ir_dh1_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0;
	
    int sensor_state = (ir_dh1_state << 3) | (ir_dh2_state << 2) | (ir_dh3_state << 1) | ir_dh4_state; // ��������״̬��ϳ�һ������
	
    // ֱ��������ʽ������ǰ200��ֱ�У���ת��
    if((sensor_state == STATE_LEFT_90_A || sensor_state == STATE_RIGHT_90_A||sensor_state == STATE_LEFT_90_B || sensor_state == STATE_RIGHT_90_B) && turn_cnt == 0)
    {
        saved_state = sensor_state;  // ��סת��״̬
        turn_cnt = 1;
    }
    if(turn_cnt > 0)
    {
        if(turn_cnt < 175) sensor_state = STATE_STRAIGHT;  // ǰ200��ֱ��
        else if(turn_cnt < 4000&&sensor_state!=STATE_LEFT_BIG&&sensor_state!=STATE_RIGHT_BIG) sensor_state = saved_state; 
        else { turn_cnt = 0; saved_state = 0; }  
        if(turn_cnt > 0) turn_cnt++; 
    }
  /*=========================================================================*
     * ״̬�жϣ�����ת�����												   *
     *=========================================================================*/
    switch (sensor_state)
    {
        case STATE_CROSS:// ����·�ڴ���
			turn_diff = 0;
            break;
        case STATE_LEFT_90_A: // ��ֱ����
		case STATE_LEFT_90_B: // ��ֱ����
            turn_diff = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // ��ֱ����
		case STATE_RIGHT_90_B: // ��ֱ����
            turn_diff = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://�����
            turn_diff = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG://�Ҵ���
            turn_diff = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL://��΢��
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL://��΢��
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT://ֱ��
            turn_diff = 0;
            break;
        case STATE_LOST://���ߴ���
            if (last_state == STATE_LEFT_SMALL) turn_diff = TurnMidAngle;//������ת
			else if (last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;//������ת
			else if(last_state == STATE_LEFT_BIG ) turn_diff = TurnMaxAngle;//������ת
			else if(last_state == STATE_RIGHT_BIG ) turn_diff = -TurnMaxAngle;//������ת
            break;
        default: // δ����״̬��ֱ��
            turn_diff = 0;
            break;
    }
	//���洫����״̬
	if(sensor_state!=STATE_LOST)
	{
		last_state=sensor_state;
	}
    // ת���ٶ�Խ�󣬻����ٶ�Խ��
	if(fabs(turn_diff)<ForwardLimit)
	{
		base_speed_mm = BaseSpeed - (BaseSpeed * (fabs(turn_diff) / ForwardLimit));
	}
	else base_speed_mm=0;
    /*========================================================================*
     * ���õ��Ŀ���ٶȣ���-ת����٣���+ת����٣���λ��mm/s��                   *
     *=========================================================================*/
	left_motor_speed = 0.001f * (base_speed_mm - turn_diff); 
    right_motor_speed = 0.001f * (base_speed_mm + turn_diff);
    // ��ֵ�����Ŀ���ٶ�
    MotorA.Target_Encoder = left_motor_speed;//����
    MotorB.Target_Encoder = right_motor_speed;//�ҵ��
}









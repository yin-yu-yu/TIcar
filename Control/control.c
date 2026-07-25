/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��5.7
�޸�ʱ�䣺2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update��2021-04-29

All rights reserved
***********************************************/
#include "control.h"
#include "IR_Module.h"

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

Encoder OriginalEncoder; 					//������ԭʼ����   
Motor_parameter MotorA,MotorB;				//���ҵ����ر���
float Velocity_KP=400,Velocity_KI=400;	
int Run_Mode=1;//С������ģʽ
u8 Flag_Stop=1;//С��������־λ
void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			
			Key();
			LED_Flash(100);
			Get_Velocity_From_Encoder(-Get_Encoder_countA,-Get_Encoder_countB);
			Get_Encoder_countA=Get_Encoder_countB=0;
			if(Run_Mode==0)
			{
				Get_RC();         //Handle the APP remote commands //����APPң������
			}else if(Run_Mode==1){
				IRDM_line_inspection();
			}
//			//�������ҵ����Ӧ��PWM
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
			if(!Flag_Stop)
			{
				Set_PWM(-MotorA.Motor_Pwm,MotorB.Motor_Pwm);
			}else Set_PWM(0,0);
		}
    }
}

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
�������ܣ���ȡ��������ת�����ٶ�
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	
	//Retrieves the original data of the encoder
	//��ȡ��������ԭʼ����
	float Encoder_A_pr,Encoder_B_pr; 
	OriginalEncoder.A=-Encoder1;	
	OriginalEncoder.B=-Encoder2;	
	Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;
	//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
	MotorA.Current_Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Perimeter/(EncoderMultiples*ENCODER_RESOLUTION*MOTOR_GEAR_RATIO);  
	MotorB.Current_Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Perimeter/(EncoderMultiples*ENCODER_RESOLUTION*MOTOR_GEAR_RATIO);  
}
//�˶�ѧ��⣬��x��y���ٶȵõ����������ٶ�,Vx��m/s,Vz��λ�Ƕ�/s(�Ƕ���)
void Get_Target_Encoder(float Vx,float Vz)
{
	float amplitude=3.5f; //Wheel target speed limit //����Ŀ���ٶ��޷�
	if(Vx<0) Vz=-Vz;
	else     Vz=Vz;
	//Inverse kinematics //�˶�ѧ���
	 MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //��������ֵ�Ŀ���ٶ�
	 MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //��������ֵ�Ŀ���ٶ�
}


/**************************************************************************
Function: Absolute value function
Input   : a��Number to be converted
Output  : unsigned int
�������ܣ�����ֵ����
��ڲ�����a����Ҫ�������ֵ����
����  ֵ���޷�������
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//��ѹ����7V��ʹ�ܿ��ش�
//	{
//		temp = 1;
//	}
	return temp;			
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
�������ܣ�����PWM��ֵ
��ڲ���: IN���������  max���޷����ֵ  min���޷���Сֵ 
����  ֵ���޷����ֵ
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//����ʽPI������
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//������һ��ƫ�� 
	 return Pwm;                         					//�������
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//����ʽPI������
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//������һ��ƫ�� 
	 return Pwm;                         					//�������
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	 switch(Flag_Direction) //Handle direction control commands //���������������
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //����ת 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //����ת	
//	}
	
//	//Z-axis data conversion //Z������ת��
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//�������ṹС��ת��Ϊǰ��ת��Ƕ�
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
//	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z=Move_Z*RC_Velocity/200;
	}		
	
	//Unit conversion, mm/s -> m/s
  //��λת����mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//�õ�����Ŀ��ֵ�������˶�ѧ����
	Get_Target_Encoder(Move_X,Move_Z);
}

/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
�������ܣ������޸�С������״̬
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=key_scan(200);//click_N_Double(50);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
	}		//��������С������ͣ
	else if(tmp==2)
	{
		Run_Mode++;
		Run_Mode%=2;
	}
}

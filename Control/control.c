/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "control.h"
#include "IR_Module.h"

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
float Velocity_KP=400,Velocity_KI=400;	
int Run_Mode=1;//小车运行模式
u8 Flag_Stop=1;//小车停止标志位
void TIM_Diff(void);  // 前向声明

void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
		switch(Car_Mode)
		{
			case Diff_Car:  TIM_Diff();  break;
			case Akm_Car:   TIM_Diff();  break;
		}
	}
}
void TIM_Diff(void)
{
			Get_Velocity_From_Encoder(Get_Encoder_countA,Get_Encoder_countB);
			Get_Encoder_countA=Get_Encoder_countB=0;
			if(Run_Mode==0)
			{
				Get_RC();         //处理APP遥控命令
			}else if(Run_Mode==1){
				IRDM_line_inspection();
			}
//			//计算左右电机对应的PWM
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
			if(!Flag_Stop)
			Set_PWM(MotorA.Motor_Pwm,MotorB.Motor_Pwm);
			else
			Set_PWM(0,0);
			Key();
}
/**************************************************************************
功能：从编码器原始数据转换为速度
输入：无
输出：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	
	//获取编码器原始数据
	float Encoder_A_pr,Encoder_B_pr; 
	OriginalEncoder.A=-Encoder1;	
	OriginalEncoder.B=-Encoder2;	
	Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;
	//将编码器原始数据转换为轮子速度，单位m/s
	MotorA.Current_Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Perimeter/(EncoderMultiples*ENCODER_RESOLUTION*MOTOR_GEAR_RATIO);  
	MotorB.Current_Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Perimeter/(EncoderMultiples*ENCODER_RESOLUTION*MOTOR_GEAR_RATIO);  
}
//运动学解算，由x、y轴速度得到左右轮速度,Vx单位m/s,Vz单位角度/s(角度制)
void Get_Target_Encoder(float Vx,float Vz)
{
	float amplitude=3.5f; //车轮目标速度限幅
	if(Vx<0) Vz=-Vz;
	else     Vz=Vz;
	//逆运动学解算
	 MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //左轮的目标速度
	 MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //右轮的目标速度
}


/**************************************************************************
功能：绝对值函数
输入：a：需要取绝对值的数
输出：无符号整数
**************************************************************************/
int myabs(int a)
{
	  int temp;
	  if(a<0)   temp=-a;
	  else temp=a;
	  return temp;
}

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//电压低于7V且使能开关打开
//	{
//		temp = 1;
//	}
	return temp;
}
/**************************************************************************
功能：限制PWM幅值
输入：IN：输入值  max：限幅最大值  min：限幅最小值 
输出：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT;
	if(IN>max)     	  OUT = max;
	else if(IN<min)	  OUT = min;
	else      		  OUT = IN;
	return OUT;
}
/**************************************************************************
功能：增量式PI控制器
输入参数：编码器数值、目标速度
返回值：电机PWM
增量式离散PID公式 
pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)：本次偏差 
e(k-1)：上一次的偏差  以此类推 
pwm：增量输出
由于速度控制闭环系统特性，只使用PI控制
pwm+=Kp[e(k)-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}
/**************************************************************************
功能：处理APP通过串口2发送过来的命令
输入：无
输出：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	 switch(Flag_Direction) //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;     	 		 Move_Z=-PI/2;   	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;   		 break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=PI/2;   	 break;
			case 7:      Move_X=0;          		 Move_Z=PI/2;   	 break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //右自转	
//	}
	
//	//Z轴数据转换
	if(Car_Mode==Akm_Car)
	{
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
//	  if(Move_X<0) Move_Z=-Move_Z; //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/200;
	}		
	
	//单位转换：mm/s -> m/s
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//得到控制目标值并进行运动学解算
	Get_Target_Encoder(Move_X,Move_Z);
}

/**************************************************************************
功能：按键修改小车运行状态
输入：无
输出：无
**************************************************************************/
void Key(void)
{
	u8 tmp;
	tmp=key_scan(CONTROL_FREQUENCY);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
	}		//按键控制小车启停
	else if(tmp==2)
	{
		Run_Mode++;
		if(Run_Mode==2)	Run_Mode=0;
	}
}

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

/* ========================================================================
 * 新旧 API 切换开关
 *   #define USE_NEW_API → 使用新架构 (SM_Run + MotionControl_Update)
 *   注释掉           → 使用旧架构 (TIM_Diff, 增量PI)
 * ======================================================================== */
#define USE_NEW_API

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

Encoder OriginalEncoder;                    //编码器原始数据
Motor_parameter MotorA,MotorB;              //左右电机相关变量
float Velocity_KP=400,Velocity_KI=400;
int Run_Mode=1;//小车运行模式
u8 Flag_Stop=1;//小车停止标志位
void TIM_Diff(void);  // 前向声明

void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
#ifdef USE_NEW_API
        /* ---- 新架构：状态机 (200Hz) ---- */
        switch(Car_Mode)
        {
            case Diff_Car:
                SM_Run();                     /* 状态机 → 设运动目标     */
                MPU6050_Read(0, 0, 0);        /* IMU 数据读取 (200Hz)   */
                (void)MPU6050_DataReady();    /* 标记新数据可用         */
                MotionControl_Update();       /* 编码器 → PID → PWM    */
                Encoder_Reset();         /* 清零编码器（下周期用）  */
                break;
            case Akm_Car:
                TIM_Diff();              /* 阿克曼暂用旧代码        */
                break;
            default:
                TIM_Diff();
                break;
        }
        Key();
        LED_Flash(100);
#else
        /* ---- 旧架构（原始代码）---- */
        switch(Car_Mode)
        {
            case Diff_Car:  TIM_Diff();  break;
            case Akm_Car:   TIM_Diff();  break;
        }
#endif
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
//          //计算左右电机对应的PWM
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


/**************************************************************************
功能：运动学分解，由x、z轴速度得到左右轮速度
Vx：x轴线速度(m/s)，Vz：z轴角速度(rad/s)
**************************************************************************/
void Get_Target_Encoder(float Vx, float Vz)
{
    float amplitude = 3.5f; // 电机目标速度限幅
    if (Vx < 0) Vz = -Vz;
    else        Vz = Vz;
    // 运动学分解
    MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f;
    MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f;
}


/**************************************************************************
功能：绝对值函数
输入：a：需要转换的数
输出：无符号整数
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
//      if(Voltage>700&&EN==0)//电压大于7V且使能开关打开
//      {
//              temp = 1;
//      }
        return temp;
}
/**************************************************************************
功能：PWM限幅
输入：IN(输入)；max(最大值)；min(最小值)
输出：OUT(限幅后的值)
**************************************************************************/
float PWM_Limit(float IN,float max,float min)
{
        float OUT = IN;
        if(OUT>max) OUT = max;
        if(OUT<min) OUT = min;
        return OUT;
}
/**************************************************************************
功能：增量PI控制器
输入：编码器值（当前速度）、目标速度
输出：PWM
增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)：本次偏差
e(k-1)：上一次的偏差  以此类推
pwm：输出
由于速度控制闭环系统，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{
         static float Bias,Pwm,Last_bias;
         Bias=Target-Encoder;                                   //计算偏差
         Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;    //增量式PI控制器
        if(Flag_Stop) Pwm=0;
         if(Pwm>7800)Pwm=7800;
         if(Pwm<-7800)Pwm=-7800;
         Last_bias=Bias;                                        //保存上一次偏差
         return Pwm;                                            //输出
}


int Incremental_PI_Right (float Encoder,float Target)
{
         static float Bias,Pwm,Last_bias;
         Bias=Target-Encoder;                                   //计算偏差
         Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;    //增量式PI控制器
        if(Flag_Stop) Pwm=0;
         if(Pwm>7800)Pwm=7800;
         if(Pwm<-7800)Pwm=-7800;
         Last_bias=Bias;                                        //保存上一次偏差
         return Pwm;                                            //输出
}
/**************************************************************************
功能：通过串口2处理APP发送的命令
输入：无
输出：无
**************************************************************************/
void Get_RC(void)
{
        u8 Flag_Move=1;
         switch(Flag_Direction) //处理方向控制命令
         {
                        case 1:      Move_X=+RC_Velocity;       Move_Z=0;         break;
                        case 2:      Move_X=+RC_Velocity;       Move_Z=-PI/2;       break;
                        case 3:      Move_X=0;                           Move_Z=-PI/2;       break;
                        case 4:      Move_X=-RC_Velocity;       Move_Z=-PI/2;     break;
                        case 5:      Move_X=-RC_Velocity;       Move_Z=0;         break;
                        case 6:      Move_X=-RC_Velocity;       Move_Z=+PI/2;     break;
                        case 7:      Move_X=0;                                   Move_Z=+PI/2;     break;
                        case 8:      Move_X=+RC_Velocity;       Move_Z=+PI/2;     break;
                        default:     Move_X=0;               Move_Z=0;         break;
         }
         if     (Flag_Left ==1)  Move_Z= PI/2; //左转
         else if(Flag_Right==1)  Move_Z=-PI/2; //右转
//      }

//      //Z轴数据转换
        if(Car_Mode==Akm_Car)
        {
                //阿克曼结构小车转换为前轮转向角度
                Move_Z=Move_Z*2/9;
        }
        else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
        {
//        if(Move_X<0) Move_Z=-Move_Z; //差速控制原理需要此处理
                Move_Z=Move_Z*RC_Velocity/200;
        }

        //单位转换，mm/s -> m/s
        Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;

        //得到控制目标值，进行运动学分解
        Get_Target_Encoder(Move_X,Move_Z);
}

/**************************************************************************
功能：按修改小车运行状态
输入：无
输出：无
**************************************************************************/
void Key(void)
{
        u8 tmp,tmp2;
        tmp=key_scan(200);//click_N_Double(50);
        if(tmp==1)
        {
                Flag_Stop=!Flag_Stop;
        }               //单机小车启停
        else if(tmp==2)
        {
                Run_Mode++;
                Run_Mode%=2;
        }
}

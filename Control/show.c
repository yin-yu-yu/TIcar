/***********************************************
公司：鲸鱼科技（东莞）有限公司
品牌：WHEELTEC
网站：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "show.h"
#include "control.h"
#include "DataScope_DP.h"
/**************************************************************************
功能：OLED 显示
输入：无
输出：无
**************************************************************************/
void oled_show(void)
{
     memset(OLED_GRAM,0, 128*8*sizeof(u8)); //GRAM清零但不影响刷新，防止残影
        //=============第一行显示小车模式=======================//
	
             if(Car_Mode==0)   OLED_ShowString(0,0,"Mec ");
        else if(Car_Mode==1)   OLED_ShowString(0,0,"Omni");
        else if(Car_Mode==2)   OLED_ShowString(0,0,"AKM ");
        else if(Car_Mode==3)   OLED_ShowString(0,0,"Diff");
        else if(Car_Mode==4)   OLED_ShowString(0,0,"4WD ");
		else if(Car_Mode==5)   OLED_ShowString(0,0,"Tank");
	
	    if(Run_Mode==0)   OLED_ShowString(90,0,"APP");
		else if(Run_Mode==1)   OLED_ShowString(90,0,"IRF");
		OLED_ShowNumber(0,10,ir_dh1_state,1,12);
		OLED_ShowNumber(20,10,ir_dh2_state,1,12);
		OLED_ShowNumber(40,10,ir_dh3_state,1,12);
		OLED_ShowNumber(60,10,ir_dh4_state,1,12);
		OLED_ShowString(00,20,"VZ");
		if( Move_Z<0)    OLED_ShowString(48,20,"-");
		if(Move_Z>=0)    OLED_ShowString(48,20,"+");
		OLED_ShowNumber(56,20, myabs((int)(Move_Z*1000)),4,12);
        //=============第三行显示左电机PWM和速度=======================//
                              OLED_ShowString(00,30,"L");
        if((MotorA.Target_Encoder*1000)<0)          OLED_ShowString(16,30,"-"),
                                                  OLED_ShowNumber(26,30,myabs((int)(MotorA.Target_Encoder*1000)),4,12);
        if((MotorA.Target_Encoder*1000)>=0)       OLED_ShowString(16,30,"+"),
                              OLED_ShowNumber(26,30,myabs((int)(MotorA.Target_Encoder*1000)),4,12);

        if(MotorA.Current_Encoder<0)   OLED_ShowString(60,30,"-");
        if(MotorA.Current_Encoder>=0)    OLED_ShowString(60,30,"+");
                              OLED_ShowNumber(68,30,myabs((int)(MotorA.Current_Encoder*1000)),4,12);
                                                    OLED_ShowString(96,30,"mm/s");

        //=============第四行显示右电机PWM和速度=======================//
                              OLED_ShowString(00,40,"R");
        if((MotorB.Target_Encoder*1000)<0)         OLED_ShowString(16,40,"-"),
                                                    OLED_ShowNumber(26,40,myabs((int)(MotorB.Target_Encoder*1000)),4,12);
        if((MotorB.Target_Encoder*1000)>=0)    		OLED_ShowString(16,40,"+"),
													OLED_ShowNumber(26,40,myabs((int)(MotorB.Target_Encoder*1000)),4,12);

        if(MotorB.Current_Encoder<0)    OLED_ShowString(60,40,"-");
        if(MotorB.Current_Encoder>=0)   OLED_ShowString(60,40,"+");
                              OLED_ShowNumber(68,40,myabs((int)(MotorB.Current_Encoder*1000)),4,12);
                                                    OLED_ShowString(96,40,"mm/s");

        //=============第五行显示电压和开关状态=======================//
                              OLED_ShowString(0,50,"V");
                                                    OLED_ShowString(30,50,".");
                                                    OLED_ShowString(64,50,"V");
                                                    OLED_ShowNumber(19,50,(int)Voltage,2,12);
                                                    OLED_ShowNumber(39,50,(u16)(Voltage*10)%10,2,12);
        if(Flag_Stop)         OLED_ShowString(95,50,"OFF");
        if(!Flag_Stop)        OLED_ShowString(95,50,"ON ");

        //=============刷新=======================//
        OLED_Refresh_Gram();
		
}
/**************************************************************************
功能：向 APP 发送数据
输入：无
输出：无
**************************************************************************/
void APP_Show(void)
{
  static u8 flag;
    int Encoder_Left_Show,Encoder_Right_Show,Voltage_Show;
    Voltage_Show=(Voltage-1000)*2/3;        if(Voltage_Show<0)Voltage_Show=0;if(Voltage_Show>100) Voltage_Show=100;   //对电压数据进行处理
    Encoder_Right_Show=Velocity_Right*1.1; if(Encoder_Right_Show<0) Encoder_Right_Show=-Encoder_Right_Show;           //对编码器数据取绝对值处理
    Encoder_Left_Show=Velocity_Left*1.1;  if(Encoder_Left_Show<0) Encoder_Left_Show=-Encoder_Left_Show;
    flag=!flag;
    if(PID_Send==1)         //发送PID数据,在APP波形界面显示
    {
        printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d:}$",(int)Velocity_KP,(int)Velocity_KI,(int)BaseSpeed,(int)(Turn90Angle) ,(int) (TurnMaxAngle),(int)(TurnMidAngle),(int)(TurnMinAngle),(int)ForwardLimit,(int)0);   
        PID_Send=0;
    }
   else if(flag==0)     // 发送电池电压、速度、角度等参数到APP首页显示
        printf("{A%d:%d:%d:%d}$",(int)Encoder_Left_Show,(int)Encoder_Right_Show,(int)Voltage_Show,(int)0); //打印到APP显示
     else                               //发送小车姿态角，在波形界面显示
      printf("{B%d:%d:%d}$",(int)0,(int)0,(int)0); //x、y、z轴角度 在APP数据显示区域
                                                                                                                    //可按此格式添加更多通道显示数据
}
/**************************************************************************
功能：虚拟示波器向上位机发送数据（已关闭显示）
输入：无
输出：无
**************************************************************************/
void DataScope(void)
{
    u8 i;//计数变量
    float Vol;                              //电压变量
    unsigned char Send_Count; //定义需要发送的数据个数
 //   Vol=(float)Voltage/100;
    DataScope_Get_Channel_Data( 0, 1 );       //显示角度 单位：（度）
    DataScope_Get_Channel_Data( 0, 2 );         //显示小车移动的距离 单位：CM
    DataScope_Get_Channel_Data( 0, 3 );                 //显示电池电压 单位：V
//      DataScope_Get_Channel_Data( 0 , 4 );
//      DataScope_Get_Channel_Data(0, 5 ); //把需要显示的数据替换0即可
//      DataScope_Get_Channel_Data(0 , 6 );//把需要显示的数据替换0即可
//      DataScope_Get_Channel_Data(0, 7 );
//      DataScope_Get_Channel_Data( 0, 8 );
//      DataScope_Get_Channel_Data(0, 9 );
//      DataScope_Get_Channel_Data( 0 , 10);
    Send_Count = DataScope_Data_Generate(3);
    for(i = 0 ; i < Send_Count; i++)
    {
//        uart0_send_char(DataScope_OutPut_Buffer[i]);
    }
}

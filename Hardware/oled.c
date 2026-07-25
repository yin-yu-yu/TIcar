#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"

#include "board.h"
#include "stdio.h"

uint8_t OLED_GRAM[128][8];
/**************************************************************************
功能：刷新OLED屏幕
输入：无
输出：无
**************************************************************************/
void OLED_Refresh_Gram(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}
/**************************************************************************
功能：向OLED写入一个字节
输入：dat:要写入的数据/命令, cmd:数据/命令标志 0表示命令;1表示数据
输出：无
**************************************************************************/
void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
	uint8_t i;
	if(cmd)
	  OLED_RS_Set();
	else
	  OLED_RS_Clr();
	for(i=0;i<8;i++)
	{
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;
	}
	OLED_RS_Set();
}
/**************************************************************************
功能：开启OLED显示
输入：无
输出：无
**************************************************************************/
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
/**************************************************************************
功能：关闭OLED显示
输入：无
输出：无
**************************************************************************/
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
/**************************************************************************
功能：清屏函数，清完屏后整个屏幕是黑色的，和没点亮一样
输入：无
输出：无
**************************************************************************/
void OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;
	OLED_Refresh_Gram(); //更新显示
}
/**************************************************************************
功能：画点
输入：x,y:起点坐标; t:1,填充,0,清空
输出：无
**************************************************************************/
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>127||y>63)return;//超出范围了.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;
}
/**************************************************************************
功能：在指定位置显示一个字符,包括部分字符
输入：x,y:起点坐标; len:数字的位数; size:字体大小; mode:0,反白显示,1,正常显示
输出：无
**************************************************************************/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' '; //得到偏移后的值
    for(t=0;t<size;t++)
    {
		if(size==12)temp=oled_asc2_1206[chr][t];  //调用1206字体
		else temp=oled_asc2_1608[chr][t];          //调用1608字体
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}
    }
}
/**************************************************************************
功能：求m的n次方
输入：m：底数，n：幂次数
输出：无
**************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}

/**************************************************************************
功能：显示数字
输入：x,y:起点坐标; len:数字的位数; size:字体大小; mode:模式, 0,填充模式, 1,叠加模式; num:数值(0~4294967295);
输出：无
**************************************************************************/
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1;

		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1);
	}
}
/**************************************************************************
功能：显示字符串
输入：x,y:起点坐标; *p:字符串起始地址
输出：无
**************************************************************************/
void OLED_ShowString(uint8_t x,uint8_t y,const char *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while(*p!='\0')
    {
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);
        x+=8;
        p++;
    }
}
/**************************************************************************
功能：初始化OLED
输入：无
输出：无
**************************************************************************/
void OLED_Init(void)
{


	OLED_RST_Clr();
	delay_ms(120);

	OLED_RST_Set();

	OLED_WR_Byte(0xAE,OLED_CMD); //关闭显示
	OLED_WR_Byte(0xD5,OLED_CMD); //设置时钟分频因子,振荡频率
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],分频因子;[7:4],振荡频率
	OLED_WR_Byte(0xA8,OLED_CMD); //设置驱动路数
	OLED_WR_Byte(0X3F,OLED_CMD); //默认0X3F(1/64)
	OLED_WR_Byte(0xD3,OLED_CMD); //设置显示偏移
	OLED_WR_Byte(0X00,OLED_CMD); //默认为0

	OLED_WR_Byte(0x40,OLED_CMD); //设置显示开始行 [5:0],行数

	OLED_WR_Byte(0x8D,OLED_CMD); //电荷泵设置
	OLED_WR_Byte(0x14,OLED_CMD); //bit2，开启/关闭
	OLED_WR_Byte(0x20,OLED_CMD); //设置内存地址模式
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
	OLED_WR_Byte(0xA1,OLED_CMD); //段重定义设置,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	OLED_WR_Byte(0xDA,OLED_CMD); //设置COM硬件引脚配置
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]配置

	OLED_WR_Byte(0x81,OLED_CMD); //对比度设置
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;默认0X7F (亮度设置,越大越亮)
	OLED_WR_Byte(0xD9,OLED_CMD); //设置预充电周期
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //设置VCOMH 电压倍率
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	OLED_WR_Byte(0xA6,OLED_CMD); //设置显示模式;bit0:1,反相显示;0,正常显示
	OLED_WR_Byte(0xAF,OLED_CMD); //开启显示
	OLED_Clear();
}

/**************************************************************************
功能：显示汉字
输入：x: 表示显示的水平坐标; y: 表示显示的垂直坐标;
      no: 表示要显示的汉字（模）在hzk[][]数组中的行号,通过行号确定数组里面要显示的汉字,
          这里的参数font_width的值必须和取模软件取模时的点阵值大小一致;
      font_height:为使用取模软件取模时的字体高度,因为我的屏幕像素为32*128-----0~7，8页，每页4个位
输出：无
注：这种方法显示汉字一定要满足取模软件生成的字模与点阵大小相同的条件下才可以使用
**************************************************************************/
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t font_width,uint8_t font_height)
{
	 uint8_t t, i;
   for(i=0;i<(font_height/8);i++)	//font_height最大值为32，这个屏幕只有8个页（行），每页4个位
	 {
			OLED_Set_Pos(x,y+i);
			for(t=0;t<font_width;t++)		//font_width最大值为128，屏幕只有那么大
			{
					OLED_WR_Byte(Hzk16[(font_height/8)*no+i][t],OLED_DATA);
			}
	 }
}
/**************************************************************************
功能：设置屏幕显示的坐标（位置）
输入：x,y:起点坐标
输出：无
**************************************************************************/
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	 OLED_WR_Byte(0xb0+y,OLED_CMD);
	 OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	 OLED_WR_Byte((x&0x0f),OLED_CMD);
}


void OLED_RST_Clr(void)
{
DL_GPIO_clearPins(OLED_RST_PORT,OLED_RST_PIN_RST_PIN);
}
		  //RST
void OLED_RST_Set(void)
{
 DL_GPIO_setPins(OLED_RST_PORT,OLED_RST_PIN_RST_PIN);
}

void OLED_RS_Clr(void)
{

DL_GPIO_clearPins(OLED_DC_PORT,OLED_DC_PIN_DC_PIN); ////DC
}
void OLED_RS_Set(void)
{
 DL_GPIO_setPins(OLED_DC_PORT,OLED_DC_PIN_DC_PIN) ; ////DC
}

void OLED_SCLK_Clr(void)
{

DL_GPIO_clearPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
}
void OLED_SCLK_Set(void)
{
DL_GPIO_setPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
}
void OLED_SDIN_Clr(void)
{
DL_GPIO_clearPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); // //SDA
}
void OLED_SDIN_Set(void)
{
DL_GPIO_setPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); //   //SDA
}

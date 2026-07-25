#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 

#include "board.h"
#include "stdio.h"

uint8_t OLED_GRAM[128][8];	 
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷刷锟斤拷OLED锟斤拷幕
锟斤拷诓锟斤拷锟斤拷锟斤拷锟�
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/
void OLED_Refresh_Gram(void)
{
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //Set page address (0~7) //锟斤拷锟斤拷页锟斤拷址锟斤拷0~7锟斤拷
		OLED_WR_Byte (0x00,OLED_CMD);      //Set the display location - column low address //锟斤拷锟斤拷锟斤拷示位锟矫★拷锟叫低碉拷址
		OLED_WR_Byte (0x10,OLED_CMD);      //Set the display location - column height address //锟斤拷锟斤拷锟斤拷示位锟矫★拷锟叫高碉拷址   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : Dat: data/command to write, CMD: data/command flag 0, represents the command;1, represents data
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷OLED写锟斤拷一锟斤拷锟街斤拷
锟斤拷诓锟斤拷锟斤拷锟絛at:要写锟斤拷锟斤拷锟斤拷锟�/锟斤拷锟筋，cmd:锟斤拷锟斤拷/锟斤拷锟斤拷锟街� 0,锟斤拷示锟斤拷锟斤拷;1,锟斤拷示锟斤拷锟斤拷
锟斤拷锟斤拷  值锟斤拷锟斤拷
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
Function: Turn on the OLED display
Input   : none
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟斤拷OLED锟斤拷示 
锟斤拷诓锟斤拷锟斤拷锟斤拷锟�
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/  
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC command //SET DCDC锟斤拷锟斤拷
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
/**************************************************************************
Function: Turn off the OLED display
Input   : none
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟截憋拷OLED锟斤拷示 
锟斤拷诓锟斤拷锟斤拷锟斤拷锟�			  
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/  
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC command //SET DCDC锟斤拷锟斤拷
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}	
/**************************************************************************
Function: Screen clear function, clear the screen, the entire screen is black, and did not light up the same
Input   : none
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷,锟斤拷锟斤拷锟斤拷,锟斤拷锟斤拷锟斤拷幕锟角猴拷色锟侥ｏ拷锟斤拷没锟斤拷锟斤拷一锟斤拷
锟斤拷诓锟斤拷锟斤拷锟斤拷锟�		  
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/  
void OLED_Clear(void)  
{  
	uint8_t i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram(); //Update the display //锟斤拷锟斤拷锟斤拷示
}
/**************************************************************************
Function: Draw point
Input   : x,y: starting coordinate;T :1, fill,0, empty
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟斤拷 
锟斤拷诓锟斤拷锟斤拷锟絰,y :锟斤拷锟斤拷锟斤拷锟�; t:1,锟斤拷锟�,0,锟斤拷锟�			  
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/ 
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>127||y>63)return;//锟斤拷锟斤拷锟斤拷围锟斤拷.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}
/**************************************************************************
Function: Displays a character, including partial characters, at the specified position
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode :0, anti-white display,1, normal display
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷指锟斤拷位锟斤拷锟斤拷示一锟斤拷锟街凤拷,锟斤拷锟斤拷锟斤拷锟斤拷锟街凤拷
锟斤拷诓锟斤拷锟斤拷锟絰,y :锟斤拷锟斤拷锟斤拷锟�; len :锟斤拷锟街碉拷位锟斤拷; size:锟斤拷锟斤拷锟叫�; mode:0,锟斤拷锟斤拷锟斤拷示,1,锟斤拷锟斤拷锟斤拷示	   
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{      			    
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' '; //Get the offset value //锟矫碉拷偏锟狡猴拷锟街�				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //Invoke 1206 font   //锟斤拷锟斤拷1206锟斤拷锟斤拷
		else temp=oled_asc2_1608[chr][t];		      //Invoke the 1608 font //锟斤拷锟斤拷1608锟斤拷锟斤拷 	                          
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
Function: Find m to the NTH power
Input   : m: base number, n: power number
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷m锟斤拷n锟轿凤拷锟侥猴拷锟斤拷
锟斤拷诓锟斤拷锟斤拷锟絤锟斤拷锟斤拷锟斤拷锟斤拷n锟斤拷锟轿凤拷锟斤拷
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/
uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}

/**************************************************************************
Function: Displays 2 numbers
Input   : x,y: starting coordinate;Len: The number of digits;Size: font size;Mode: mode, 0, fill mode, 1, overlay mode;Num: value (0 ~ 4294967295);
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷示2锟斤拷锟斤拷锟斤拷
锟斤拷诓锟斤拷锟斤拷锟絰,y :锟斤拷锟斤拷锟斤拷锟�; len :锟斤拷锟街碉拷位锟斤拷; size:锟斤拷锟斤拷锟叫�; mode:模式, 0,锟斤拷锟侥Ｊ�, 1,锟斤拷锟斤拷模式; num:锟斤拷值(0~4294967295);	 
锟斤拷锟斤拷  值锟斤拷锟斤拷
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
Function: Display string
Input   : x,y: starting coordinate;*p: starting address of the string
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷示锟街凤拷锟斤拷
锟斤拷诓锟斤拷锟斤拷锟絰,y :锟斤拷锟斤拷锟斤拷锟�; *p:锟街凤拷锟斤拷锟斤拷始锟斤拷址 
锟斤拷锟斤拷  值锟斤拷锟斤拷
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
Function: Initialize the OLED
Input   : none
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷始锟斤拷OLED	
锟斤拷诓锟斤拷锟�: 锟斤拷 
锟斤拷锟斤拷  值锟斤拷锟斤拷
**************************************************************************/	    
void OLED_Init(void)
{ 	


	OLED_RST_Clr();
	delay_ms(120);

	OLED_RST_Set(); 
				  
	OLED_WR_Byte(0xAE,OLED_CMD); //Close display //锟截憋拷锟斤拷示
	OLED_WR_Byte(0xD5,OLED_CMD); //The frequency frequency factor, the frequency of the shock //锟斤拷锟斤拷时锟接凤拷频锟斤拷锟斤拷,锟斤拷频锟斤拷
	OLED_WR_Byte(80,OLED_CMD);   //[3:0], the frequency dividing factor;[7:4], oscillation frequency //[3:0],锟斤拷频锟斤拷锟斤拷;[7:4],锟斤拷频锟斤拷
	OLED_WR_Byte(0xA8,OLED_CMD); //Set the number of driver paths //锟斤拷锟斤拷锟斤拷锟斤拷路锟斤拷
	OLED_WR_Byte(0X3F,OLED_CMD); //Default 0x3f(1/64) //默锟斤拷0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //Setting display deviation //锟斤拷锟斤拷锟斤拷示偏锟斤拷
	OLED_WR_Byte(0X00,OLED_CMD); //Default is 0//默锟斤拷为0

	OLED_WR_Byte(0x40,OLED_CMD); //Sets the number of rows to display starting line [5:0] //锟斤拷锟斤拷锟斤拷示锟斤拷始锟斤拷 [5:0],锟斤拷锟斤拷
													
	OLED_WR_Byte(0x8D,OLED_CMD); //Charge pump setup //锟斤拷杀锟斤拷锟斤拷锟�
	OLED_WR_Byte(0x14,OLED_CMD); //Bit2, on/off //bit2锟斤拷锟斤拷锟斤拷/锟截憋拷
	OLED_WR_Byte(0x20,OLED_CMD); //Set up the memory address mode //锟斤拷锟斤拷锟节达拷锟街纺Ｊ�
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00, column address mode;01, line address mode;10. Page address mode;The default 10; //[1:0],00锟斤拷锟叫碉拷址模式;01锟斤拷锟叫碉拷址模式;10,页锟斤拷址模式;默锟斤拷10;
	OLED_WR_Byte(0xA1,OLED_CMD); //Segment redefine setting,bit0:0,0- >;0;1, 0 - & gt;127; //锟斤拷锟截讹拷锟斤拷锟斤拷锟斤拷,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //Set the COM scan direction;Bit3:0, normal mode;1, Re-define schema COM[n-1]- >;COM0;N: Number of driving paths//锟斤拷锟斤拷COM扫锟借方锟斤拷;bit3:0,锟斤拷通模式;1,锟截讹拷锟斤拷模式 COM[N-1]->COM0;N:锟斤拷锟斤拷路锟斤拷
	OLED_WR_Byte(0xDA,OLED_CMD); //Set the COM hardware pin configuration //锟斤拷锟斤拷COM硬锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]configuration //[5:4]锟斤拷锟斤拷
	 
	OLED_WR_Byte(0x81,OLED_CMD); //Contrast Settings //锟皆比讹拷锟斤拷锟斤拷
	OLED_WR_Byte(0xEF,OLED_CMD); //1~ 255; Default 0x7f (brightness Settings, the bigger the brighter) //1~255;默锟斤拷0X7F (锟斤拷锟斤拷锟斤拷锟斤拷,越锟斤拷越锟斤拷)
	OLED_WR_Byte(0xD9,OLED_CMD); //Set the pre-charging cycle //锟斤拷锟斤拷预锟斤拷锟斤拷锟斤拷锟�
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //Setting vcomh voltage multiplier//锟斤拷锟斤拷VCOMH 锟斤拷压锟斤拷锟斤拷
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //Global display; Bit0:1, open; 0, close; (white screen/black screen)//全锟斤拷锟斤拷示锟斤拷锟斤拷;bit0:1,锟斤拷锟斤拷;0,锟截憋拷;(锟斤拷锟斤拷/锟斤拷锟斤拷)
	OLED_WR_Byte(0xA6,OLED_CMD); //Settings display mode; Bit0:1, anti-phase display; 0, normal display//锟斤拷锟斤拷锟斤拷示锟斤拷式;bit0:1,锟斤拷锟斤拷锟斤拷示;0,锟斤拷锟斤拷锟斤拷示	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //Open display //锟斤拷锟斤拷锟斤拷示	 
	OLED_Clear();
}  

/**************************************************************************
Function: Display character
Input   : x: indicates the horizontal coordinates displayed; Y: the vertical coordinates that show the display;
          no: the line number in the array of the Chinese character (module) in the hzk-and "array", which is determined by the line number to determine the characters shown in the array,
          The value of the width of the font here must be consistent with the size of the dot matrix value of the use of the word mold.
          font_height: the font is high for the use of the word mold, because my screen pixels are 32hours, 128----0~ 7, and four bits per page
Output  : none
Note: this method is used to show that the Chinese character must satisfy the size of the word that the word model generates the software to generate the same size as the dot matrix
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷示锟斤拷锟斤拷	
锟斤拷诓锟斤拷锟�: x锟斤拷锟斤拷示锟斤拷示锟斤拷水平锟斤拷锟斤拷; y: 锟斤拷示锟斤拷示锟侥达拷直锟斤拷锟斤拷;
          no: 锟斤拷示要锟斤拷示锟侥猴拷锟街ｏ拷模锟介）锟斤拷hzk[][]锟斤拷锟斤拷锟叫碉拷锟叫猴拷,通锟斤拷锟叫猴拷锟斤拷确锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷要锟斤拷示锟侥猴拷锟斤拷,
              锟斤拷锟斤拷锟斤拷锟斤拷目锟絝ont_width锟斤拷值锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷模锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷模时锟侥碉拷锟斤拷值锟斤拷小一锟斤拷;
          font_height:为锟斤拷锟斤拷模锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷模时锟斤拷锟斤拷母锟�,锟斤拷锟斤拷锟揭碉拷锟斤拷锟斤拷锟斤拷为32*128-----0~7锟斤拷8页锟斤拷每页4锟斤拷位
锟斤拷锟斤拷  值锟斤拷锟斤拷
注锟解：锟斤拷锟斤拷锟街凤拷锟斤拷锟斤拷锟斤拷示锟斤拷锟斤拷一锟斤拷要锟斤拷锟斤拷锟斤拷锟斤拷模锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟缴碉拷锟街匡拷锟斤拷锟斤拷锟斤拷小锟斤拷同锟斤拷锟叫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
**************************************************************************/	    
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no,uint8_t font_width,uint8_t font_height)
{     			    
	 uint8_t t, i;
   for(i=0;i<(font_height/8);i++)	//The maximum height of font_height is 32. this screen is only 8 pages (line), four digits per page
	                                //font_height锟斤拷锟街滴�32锟斤拷锟斤拷锟斤拷锟斤拷只锟斤拷8锟斤拷页锟斤拷锟叫ｏ拷锟斤拷每页4锟斤拷位
	 {
			OLED_Set_Pos(x,y+i);	
			for(t=0;t<font_width;t++)		//The maximum value of font_width is 128. the screen is only that large 
		                              //font_width锟斤拷锟街滴�128锟斤拷锟斤拷幕只锟斤拷锟斤拷么锟斤拷
			{	
					OLED_WR_Byte(Hzk16[(font_height/8)*no+i][t],OLED_DATA);
			}		
	 }
}	 
/**************************************************************************
Function: Set the coordinates (position) displayed on the screen.
Input   : x, y: starting point coordinates
Output  : none
锟斤拷锟斤拷锟斤拷锟杰ｏ拷锟斤拷锟矫猴拷锟斤拷锟斤拷锟斤拷幕锟斤拷锟斤拷示锟斤拷锟斤拷锟疥（位锟矫ｏ拷
锟斤拷诓锟斤拷锟�: x,y :锟斤拷锟斤拷锟斤拷锟�
锟斤拷锟斤拷  值锟斤拷锟斤拷
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
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");

} 	
		  //RST
void OLED_RST_Set(void)
{
 DL_GPIO_setPins(OLED_RST_PORT,OLED_RST_PIN_RST_PIN);
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}  

void OLED_RS_Clr(void)
{

DL_GPIO_clearPins(OLED_DC_PORT,OLED_DC_PIN_DC_PIN); ////DC
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_RS_Set(void) 
{
 DL_GPIO_setPins(OLED_DC_PORT,OLED_DC_PIN_DC_PIN) ; ////DC
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
} 	

void OLED_SCLK_Clr(void)
{

DL_GPIO_clearPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SCLK_Set(void)
{
DL_GPIO_setPins(OLED_SCL_PORT,OLED_SCL_PIN_SCL_PIN); ////SCL
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SDIN_Clr(void)
{
DL_GPIO_clearPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); // //SDA
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
}   
void OLED_SDIN_Set(void) 
{
DL_GPIO_setPins(OLED_SDA_PORT,OLED_SDA_PIN_SDA_PIN); //   //SDA
//	 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
//	 __asm__("nop");
//		 __asm__("nop");
} 	

#include "OLED.h"
#include "codebase.h"
#include "stm32f401xc.h"
#include "i2c.h"

void OLED_WrCmd(uint8_t WrCmd)
{
	HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &WrCmd , 1, 10);
}

void OLED_WrDat(uint8_t WrDat)
{
	HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, &WrDat , 1, 10);
}

void OLED_Init(void)
{
	unsigned int a;
	for(a=0;a<5000;a++);
	OLED_WrCmd(0xAE);//--turn off oled panel
 	OLED_WrCmd(0x00);//---set low column address
 	OLED_WrCmd(0x10);//---set high column address
 	OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WrCmd(0xB0);
 	OLED_WrCmd(0x81);//--set contrast control register
 	OLED_WrCmd(0xFF); // Set SEG Output Current Brightness
 	OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
 	OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
 	OLED_WrCmd(0xa6);//--set normal display
 	OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
 	OLED_WrCmd(0x3f);//--1/64 duty
 	OLED_WrCmd(0xd3);//-set display offset Shift Mapping RAM Counter (0x00~0x3F)
 	OLED_WrCmd(0x00);//-not offset
	OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
 	OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WrCmd(0xd9);//--set pre-charge period
 	OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
 	OLED_WrCmd(0xda);//--set com pins hardware configuration
 	OLED_WrCmd(0x12);
 	OLED_WrCmd(0xdb);//--set vcomh
 	OLED_WrCmd(0x40);//Set VCOM Deselect Level
 	OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
 	OLED_WrCmd(0x02);//
 	OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
	OLED_WrCmd(0x14);//--set(0x10) disable
 	OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
 	OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)
 	OLED_WrCmd(0xaf);//--turn on oled panel
	OLED_Clear();//OLED清屏
}

void OLED_Clear(void)
{
  unsigned char i,n;
  for(i=0; i<8; i++)
  {
    OLED_WrCmd(0xb0+i); //设置页地址（0~7）
    OLED_WrCmd(0x00); //设置显示位置—列低地址
    OLED_WrCmd(0x10); //设置显示位置—列高地址
    for(n=0; n<128; n++)	OLED_WrDat(0x00); //写0x00到屏幕寄存器上
  }
}
 
void OLED_SetPos(uint8_t x, uint8_t y)
{
	OLED_WrCmd((0xb0+y));
	OLED_WrCmd(((x&0xf0)>>4)|0x10);
	OLED_WrCmd(x&0x0f);
}
 
//------将OLED从休眠中唤醒------
void OLED_ON(void)
{
  OLED_WrCmd(0X8D);  //设置电荷泵
  OLED_WrCmd(0X14);  //开启电荷泵
  OLED_WrCmd(0XAF);  //OLED唤醒
}
 
//------让OLED休眠 -- 休眠模式下,OLED功耗不到10uA------
void OLED_OFF(void)
{
  OLED_WrCmd(0X8D);  //设置电荷泵
  OLED_WrCmd(0X10);  //关闭电荷泵
  OLED_WrCmd(0XAE);  //OLED休眠
}
 
//--------------------------------------------------------------
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); ch[] -- 要显示的字符串; TextSize -- 字符大小(1:6*8 ; 2:8*16)
// Description    : 显示codetab.h中的ASCII字符,有6*8和8*16可选择
//--------------------------------------------------------------
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
  unsigned char c = 0,i = 0,j = 0;
  switch(TextSize)
  {
  case 1:
  {
    while(ch[j] != '\0')
    {
      c = ch[j] - 32;
      if(x > 126)
      {
        x = 0;
        y++;
      }
      OLED_SetPos(x,y);
      for(i=0; i<6; i++)
        OLED_WrDat(F6x8[c][i]);
      x += 6;
      j++;
    }
  }
  break;
  case 2:
  {
    while(ch[j] != '\0')
    {
      c = ch[j] - 32;
      if(x > 120)
      {
        x = 0;
        y++;
      }
      OLED_SetPos(x,y);
      for(i=0; i<8; i++)
        OLED_WrDat(F8x16[c*16+i]);
      OLED_SetPos(x,y+1);
      for(i=0; i<8; i++)
        OLED_WrDat(F8x16[c*16+i+8]);
      x += 8;
      j++;
    }
  }
  break;
  }
}
 
//****************功能描述： 显示6*8或8*16的5位整数   显示的坐标（x,y），y为页范围0～7****************************
 
/*例
OLED_ShowInt(0,0,0,1);   //在(0,0)处，显示6*8的"0"
OLED_ShowInt(5,4,12345,2);//在(5,4)处，显示8*16的"12345"
*/
void OLED_ShowInt(unsigned char x, unsigned char y, int Data, unsigned char TextSize)
{
  unsigned char temp;
  OLED_SetPos(x,y);
  switch(TextSize)
  {
  case 1:
  {
    if(Data<0)
    {
      OLED_ShowChar(x,y,'-',1);
      x+=6;
      Data=-Data;
    }
    //接下来要显示正数，清空上一次显示负数的个位
    //负数比正数多一个负号，额外占了一个显示位
    OLED_ShowChar(x+30,y,' ',1);
 
    temp=Data/10000;
    OLED_ShowChar(x,y,(temp+'0'),1);
 
    Data%=10000;
    temp=Data/1000;
    OLED_ShowChar(x+6,y,(temp+'0'),1);
 
    Data%=1000;
    temp=Data/100;
    OLED_ShowChar(x+12,y,(temp+'0'),1);
 
    Data%=100;
    temp=Data/10;
    OLED_ShowChar(x+18,y,(temp+'0'),1);
 
    Data%=10;
    temp=Data;
    OLED_ShowChar(x+24,y,(temp+'0'),1);
  }
  break;
  case 2:
  {
    if(Data<0)
    {
      OLED_ShowChar(x,y,'-',2);
      x+=8;
      Data=-Data;
    }
    //接下来要显示正数，清空上一次显示负数的个位
    //负数比正数多一个负号，额外占了一个显示位
    OLED_ShowChar(x+40,y,' ',2);
 
    temp=Data/10000;
    OLED_ShowChar(x,y,(temp+'0'),2);
 
    Data%=10000;
    temp=Data/1000;
    OLED_ShowChar(x+8,y,(temp+'0'),2);
 
    Data%=1000;
    temp=Data/100;
    OLED_ShowChar(x+16,y,(temp+'0'),2);
 
    Data%=100;
    temp=Data/10;
    OLED_ShowChar(x+24,y,(temp+'0'),2);
 
    Data%=10;
    temp=Data;
    OLED_ShowChar(x+32,y,(temp+'0'),2);
  }
  break;
  }
}
 
/***************功能描述：显示6*8或8*16一个标准ASCII字符串	显示的坐标（x,y），y为页范围0～7****************/
/*例：  OLED_ShowChar(39,0,'A',1)*/
void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch, unsigned char TextSize)
{
  unsigned char c=0,i=0;
  c =ch-32;
  if(x>120)
  {
    x=0;
    y++;
  }
  OLED_SetPos(x,y);
  switch(TextSize)
  {
  case 1:
  {
    for(i=0; i<6; i++)
      OLED_WrDat(F6x8[c][i]);
    break;
  }
  case 2:
  {
    for(i=0; i<8; i++)
      OLED_WrDat(F8x16[c*16+i]);
    OLED_SetPos(x,y+1);
    for(i=0; i<8; i++)
      OLED_WrDat(F8x16[c*16+i+8]);
    x += 8;
    break;
  }
  }
}
 
//--------------------------------------------------------------
// Parameters     : x0,y0 -- 起始点坐标(x0:0~127, y0:0~7); x1,y1 -- 起点对角线(结束点)的坐标(x1:1~128,y1:1~8)
// Description    : 显示BMP位图
//--------------------------------------------------------------
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
  unsigned int j=0;
  unsigned char x,y;
 
  if(y1%8==0)
    y = y1/8;
  else
    y = y1/8 + 1;
  for(y=y0; y<y1; y++)
  {
    OLED_SetPos(x0,y);
    for(x=x0; x<x1; x++)
    {
      OLED_WrDat(BMP[j++]);
    }
  }
}

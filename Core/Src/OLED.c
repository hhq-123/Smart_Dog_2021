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
 	OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
 	OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
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
	OLED_Clear();//OLED����
}

void OLED_Clear(void)
{
  unsigned char i,n;
  for(i=0; i<8; i++)
  {
    OLED_WrCmd(0xb0+i); //����ҳ��ַ��0~7��
    OLED_WrCmd(0x00); //������ʾλ�á��е͵�ַ
    OLED_WrCmd(0x10); //������ʾλ�á��иߵ�ַ
    for(n=0; n<128; n++)	OLED_WrDat(0x00); //д0x00����Ļ�Ĵ�����
  }
}
 
void OLED_SetPos(uint8_t x, uint8_t y)
{
	OLED_WrCmd((0xb0+y));
	OLED_WrCmd(((x&0xf0)>>4)|0x10);
	OLED_WrCmd(x&0x0f);
}
 
//------��OLED�������л���------
void OLED_ON(void)
{
  OLED_WrCmd(0X8D);  //���õ�ɱ�
  OLED_WrCmd(0X14);  //������ɱ�
  OLED_WrCmd(0XAF);  //OLED����
}
 
//------��OLED���� -- ����ģʽ��,OLED���Ĳ���10uA------
void OLED_OFF(void)
{
  OLED_WrCmd(0X8D);  //���õ�ɱ�
  OLED_WrCmd(0X10);  //�رյ�ɱ�
  OLED_WrCmd(0XAE);  //OLED����
}
 
//--------------------------------------------------------------
// Parameters     : x,y -- ��ʼ������(x:0~127, y:0~7); ch[] -- Ҫ��ʾ���ַ���; TextSize -- �ַ���С(1:6*8 ; 2:8*16)
// Description    : ��ʾcodetab.h�е�ASCII�ַ�,��6*8��8*16��ѡ��
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
 
//****************���������� ��ʾ6*8��8*16��5λ����   ��ʾ�����꣨x,y����yΪҳ��Χ0��7****************************
 
/*��
OLED_ShowInt(0,0,0,1);   //��(0,0)������ʾ6*8��"0"
OLED_ShowInt(5,4,12345,2);//��(5,4)������ʾ8*16��"12345"
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
    //������Ҫ��ʾ�����������һ����ʾ�����ĸ�λ
    //������������һ�����ţ�����ռ��һ����ʾλ
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
    //������Ҫ��ʾ�����������һ����ʾ�����ĸ�λ
    //������������һ�����ţ�����ռ��һ����ʾλ
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
 
/***************������������ʾ6*8��8*16һ����׼ASCII�ַ���	��ʾ�����꣨x,y����yΪҳ��Χ0��7****************/
/*����  OLED_ShowChar(39,0,'A',1)*/
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
// Parameters     : x0,y0 -- ��ʼ������(x0:0~127, y0:0~7); x1,y1 -- ���Խ���(������)������(x1:1~128,y1:1~8)
// Description    : ��ʾBMPλͼ
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

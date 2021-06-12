#ifndef OLED_H
#define OLED_H
#include "stm32f401xc.h"

void OLED_WrCmd(uint8_t WrCmd);
void OLED_WrDat(uint8_t WrDat);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetPos(uint8_t x, uint8_t y);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowInt(unsigned char x, unsigned char y, int Data, unsigned char TextSize);
void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch, unsigned char TextSize);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);

#endif

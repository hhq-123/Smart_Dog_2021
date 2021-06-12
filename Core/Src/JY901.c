#include "JY901.h"
#include "stm32f401xc.h"
#include "i2c.h"

void JY901_RDDat(uint8_t WrCmd)
{
	HAL_I2C_Mem_Read(&hi2c1, 0x78, 0x00, 1, &WrCmd , 1, 10);
}
#include "JY901.h"
#include "stm32f401xc.h"
#include "i2c.h"
#include "usbd_cdc_if.h"

unsigned char chrTemp[30];


JY901_Angle IMU;

void ShortToChar(short sData,unsigned char cData[])
{
	cData[0]=sData&0xff;
	cData[1]=sData>>8;
}
short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}


void JY901_RDDat(JY901_Angle *IMUx)
{
	//HAL_I2C_Mem_Read(&hi2c1, 0x50, AX, I2C_MEMADD_SIZE_8BIT, &chrTemp[0] , 24, 10);
	//HAL_I2C_Mem_Read(&hi2c1, 0x78, AX, I2C_MEMADD_SIZE_8BIT, &chrTemp[0] , 24, 10);
	HAL_I2C_Mem_Read(&hi2c1, 0x50<<1, AX, I2C_MEMADD_SIZE_8BIT, &chrTemp[0] , 24, 0xff);
	(*IMUx).a[0] = (float)CharToShort(&chrTemp[0])/32768*16;
	(*IMUx).a[1] = (float)CharToShort(&chrTemp[2])/32768*16;
	(*IMUx).a[2] = (float)CharToShort(&chrTemp[4])/32768*16;
	(*IMUx).w[0] = (float)CharToShort(&chrTemp[6])/32768*2000;
	(*IMUx).w[1] = (float)CharToShort(&chrTemp[8])/32768*2000;
	(*IMUx).w[2] = (float)CharToShort(&chrTemp[10])/32768*2000;
	(*IMUx).h[0] = CharToShort(&chrTemp[12]);
	(*IMUx).h[1] = CharToShort(&chrTemp[14]);
	(*IMUx).h[2] = CharToShort(&chrTemp[16]);
	(*IMUx).Angle[0] = (float)CharToShort(&chrTemp[18])/32768*180;
	(*IMUx).Angle[1] = (float)CharToShort(&chrTemp[20])/32768*180;
	(*IMUx).Angle[2] = (float)CharToShort(&chrTemp[22])/32768*180;	
}


#include "Quadrudep_huaner.h"
#include <LobotServoController.h>
#include "math.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"


LobotServo servos[12];   //舵机ID位置结构数组

uint16_t LSCControlPeriod = 50;

double shank[4];
double ham[4];
double wai[4];

extern osSemaphoreId_t gaitControlBinarySemHandle;

double AtoV(double angle)
{
	int reAngle = 1500;
  reAngle = 500+(2000*1.0/180)*angle;
  reAngle = reAngle>2500? 2500:reAngle;
  reAngle = reAngle<500? 500:reAngle;
  return reAngle;
}

double VtoA(int val)
{
	int reAngle = 1500;
  reAngle = (180*1.0/2000)*(val-500);
  //reAngle = reAngle>180? 2000:reAngle;
  //reAngle = reAngle<0? 0:reAngle;
  return reAngle;
}


void Servo_Init(void)
{	
  servos[0].ID = 10;       
	servos[1].ID = 12;       
  servos[2].ID = 13;
	
	servos[3].ID = 11;    
  servos[4].ID = 14;       
	servos[5].ID = 15;
    
	servos[6].ID = 4;       
	servos[7].ID = 1;    
	servos[8].ID = 0;
	
	servos[9].ID = 5;    
	servos[10].ID = 3;       
	servos[11].ID = 2;    
}


void Move(double ham[],double shank[], double wai[],  uint16_t Time)
{	
	servos[0].Position = AtoV(VtoA(LF_wai_Init)-(wai[0]));
	servos[3].Position = AtoV(VtoA(RF_wai_Init)+(wai[1]));
	servos[6].Position = AtoV(VtoA(RB_wai_Init)-(wai[2]));
	servos[9].Position = AtoV(VtoA(LB_wai_Init)+(wai[3]));
		
	servos[1].Position = AtoV(VtoA(LF_ham_Init)-90+ham[0]);
	servos[2].Position = AtoV(VtoA(LF_shank_Init)+90-shank[0]);
	
	servos[4].Position = AtoV(VtoA(RF_ham_Init)+90-ham[1]);
	servos[5].Position = AtoV(VtoA(RF_shank_Init)-90+shank[1]);
	
	servos[7].Position = AtoV(VtoA(RB_ham_Init)+90-ham[2]);
	servos[8].Position = AtoV(VtoA(RB_shank_Init)-90+shank[2]);
	
	servos[10].Position =AtoV(VtoA(LB_ham_Init)-90+ham[3]);
	servos[11].Position =AtoV(VtoA(LB_shank_Init)+90-shank[3]);
	
	moveServosByArray(servos,12,Time);  //控制两个舵机，移动时间1ms,ID和位置有servos指定
}

void Move_zero(uint16_t Time)
{ 
 servos[0].Position = LF_wai_Init;
 servos[3].Position = RF_wai_Init;
 servos[6].Position = RB_wai_Init;
 servos[9].Position = LB_wai_Init;
   
 servos[1].Position = LF_ham_Init;
 servos[2].Position = LF_shank_Init;
 
 servos[4].Position = RF_ham_Init;
 servos[5].Position = RF_shank_Init;
 
 servos[7].Position =RB_ham_Init;
 servos[8].Position = RB_shank_Init;
 
 servos[10].Position =LB_ham_Init;
 servos[11].Position =LB_shank_Init;
 
 moveServosByArray(servos,12,Time);  //控制两个舵机，移动时间1ms,ID和位置有servos指定
}

double ShankAngleConverter(double theta)
{
	double temp = 0;
	temp = Shank_c-Shank_a*sin(theta);
	temp = Shank_b*Shank_b - temp*temp;
	temp = sqrt(temp);
	temp = Shank_a * cos(theta) - Shank_d + temp;
	temp = acos(temp/Shank_e);
	return temp;
}

void ik_3dof(double x[], double y[], double z[])
{
	double t[3][4];
	double phy[4];
	double tho[4];
	double beta[4];
	for(int i = 0; i<4; i++)
	{
		t[2][i] = acos((x[i] * x[i] + y[i] * y[i] + z[i] * z[i] - l3 * l3 - l2 * l2 - d * d) / (2 * l2 * l3));
		phy[i] = atan((l3 * sin(t[2][i])) / (l3 * cos(t[2][i]) + l2));
		t[1][i] = phy[i] - asin((x[i] / (sqrt(l3 * l3 + l2 * l2 + 2 * l2 * l3 * cos(t[2][i])))));
		tho[i] = sqrt((l2 * cos(t[1][i]) + l3 * cos(t[2][i] - t[1][i])) * (l2 * cos(t[1][i]) + l3 * cos(t[2][i] - t[1][i])) + d * d);
		beta[i] = atan((l2 * cos(t[1][i]) + l3 * cos(t[2][i] - t[1][i])) / d);
		t[0][i] = beta[i] - asin(y[i] / tho[i]);
		
		wai[i] = t[0][i] * RtoA;
		ham[i] = 90 - t[1][i] * RtoA;
		shank[i] = ShankAngleConverter(t[2][i]) * RtoA;
	}
}



void gait_Init(void)
{
	double inx[4] = {0,0,0,0};
	double iny[4] = {H,H,H,H};
	double inz[4] = {d, d, d ,d};
	ik_3dof(inx, iny, inz);
	//Move(ham, shank, wai, 1);
	//printf("shank%f\r\n", shank[0]);
}

void ik_Move(double x[], double y[], double z[])
{
	ik_3dof(x, y, z);
	//Move(ham, shank, wai, LSCControlPeriod);
	//Move(ham, shank, wai, 1);
	//printf("shank%f\r\n", shank[0]);
	xSemaphoreTake(gaitControlBinarySemHandle, osWaitForever);
}

void LSC_communication(void)
{
	Move(ham, shank, wai, LSCControlPeriod);
}
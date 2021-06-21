#include "test_workspace.h"
#include "Quadrudep_huaner.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "math.h"
#include "JY901.h"
#include "OLED.h"
#include "freeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

extern JY901_Angle IMU;
extern uint16_t LSCControlPeriod;
unsigned char str[100];

double Pedis_x[4];
double Pedis_y[4];
double Pedis_z[4];

#define Ts 10
int Test_t = 10;
int Test_i = 0;
int t = 0;

PId_struct_typedef PItch_PId = {1.0,0,0.2,   0,0,0};

PId_struct_typedef roll_PId = {1.0,0,0.2,   0,0,0};

double PId_algo(PId_struct_typedef *PId, double fdb) {
  PId->error = fdb;

  PId->output = PId->error * PId->kp + PId->integral * PId->ki + (PId->error - PId->last_error) * PId->kd;

  PId->integral += PId->error;
  PId->last_error = PId->error;

  if (PId->integral > 20) PId->integral = 20;
  else if (PId->integral < -20) PId->integral = -20;

  return PId->output;
}

void gait_trot(int t, double xs, double xf, double h, double r1, double r2, double r3, double r4) 
{
	double kkk =0;
	double sigma = 0.0;
	
	double zep, xep_b, xep_z;

  if (t <= (Ts /2))
  {
		//printf("t <= Ts /2\r\n");
    sigma = 4 * PI * t / Ts;
		//printf("sigma%f\n\r", sigma);
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * PI)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * PI)) + xf;
    //输出y
    Pedis_y[0] = H - zep;
    Pedis_y[1] = H + zep * kkk;
    Pedis_y[2] = H - zep;
    Pedis_y[3] = H + zep * kkk;
		//printf("Pedis_y:%f ",Pedis_y[0]);
    //输出x
    Pedis_x[0] = xep_z * r1;
    Pedis_x[1] = xep_b * r2;
    Pedis_x[2] = xep_z * r3;
    Pedis_x[3] = xep_b * r4;
		//printf("Pedis_x:%f ",Pedis_x[0]);
    Pedis_z[0] = d;
    Pedis_z[1] = d;
    Pedis_z[2] = d;
    Pedis_z[3] = d;
		//printf("Pedis_z:%f \r\n",Pedis_z[0]);
  }
  else if (t > (Ts / 2) && t <= Ts)
  {
    sigma = 4 * PI * (t - Ts /2 ) / (Ts);
		
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * PI)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * PI)) + xf;
    //输出y
    Pedis_y[0] = H + zep * kkk;
    Pedis_y[1] = H - zep;
    Pedis_y[2] = H + zep * kkk;
    Pedis_y[3] = H - zep;
    //输出x
    Pedis_x[0] = xep_b * r1;
    Pedis_x[1] = xep_z * r2;
    Pedis_x[2] = xep_b * r3;
    Pedis_x[3] = xep_z * r4;
    Pedis_z[0] = d;
    Pedis_z[1] = d;
    Pedis_z[2] = d;
    Pedis_z[3] = d;

  }
  else {
    Pedis_x[0] = 0;
		Pedis_x[1] = 0;
		Pedis_x[2] = 0;
		Pedis_x[3] = 0;

		Pedis_y[0] = H;
		Pedis_y[1] = H;
		Pedis_y[2] = H;
		Pedis_y[3] = H;
  }
	return;
}



void gait_walk(double t, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
	double sigma = 0.0;
	double zep, xep_b, xep_z;
	int   kkk =0;
	int Zero = -40;
	
  double faai = 0.2;
  double k_walk=0;
	int wei=13;
  //前面加上定义yep(抬腿）,wei（侧摆）,ww(为机器人的宽)，xep-zs支撑相的移动
  double S = 4 * (xf - xs) / 3; //S为步长，相对地面的步长是相对身体步长的4/3
  double yep, xep_zs;
  
  if (t <= Ts * faai) //faai=0.2
  {
    sigma = 2 * PI * t / (faai * Ts); //0~2*PI
    yep = h * (1 - cos(sigma)) / 2; //0~h~0
    xep_b = 0.75 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~3S/4
    xep_z = -0.25 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~-S/4
    //右前1
    Pedis_y[0] = H - yep;        Pedis_y[1] = H+ yep*k_walk;         Pedis_y[2] = H+ yep*k_walk;         Pedis_y[3] = H+ yep*k_walk;
    Pedis_x[0] = xep_b - S / 4;  Pedis_x[1] = xep_z + S / 4; Pedis_x[2] = xep_z - S/4; Pedis_x[3] = xep_z + S/4 ;
    Pedis_z[0] = d + wei;      Pedis_z[1] = d - wei;     Pedis_z[2] = d - wei;     Pedis_z[3] = d + wei;

    xep_zs = 0.05 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~S/20
    Pedis_x[0] = Pedis_x[0] + xep_zs;  Pedis_x[1] = Pedis_x[1] + xep_zs; Pedis_x[2] = Pedis_x[2] + xep_zs; Pedis_x[3] = Pedis_x[3] + xep_zs;
  }
  else if (t > Ts * faai && t <= Ts * faai * 1.5)
  {
    //四足支撑相
    sigma = 2 * PI * (t - Ts * faai) / (faai * Ts * 0.5); //0~2*PI
    zep = 2 * wei * ((sigma - sin(sigma)) / (2 * PI)); //0~2*wei
    Pedis_y[0] = H;        Pedis_y[1] = H;        Pedis_y[2] = H;        Pedis_y[3] = H;
    Pedis_x[0] = S / 2;      Pedis_x[1] = 0;        Pedis_x[2] = -S / 2;     Pedis_x[3] = 0;
    Pedis_z[0] = d + wei - zep;     Pedis_z[1] = d - wei + zep;    Pedis_z[2] = d - wei + zep;    Pedis_z[3] = d + wei - zep;

    xep_zs = 0.05 * S - 0.1 * S * ((sigma - sin(sigma)) / (2 * PI)); //S/20~-S/20
    Pedis_x[0] = Pedis_x[0] + xep_zs;  Pedis_x[1] = Pedis_x[1] + xep_zs; Pedis_x[2] = Pedis_x[2] + xep_zs; Pedis_x[3] = Pedis_x[3] + xep_zs;
  }
  else if (t > Ts * faai * 1.5 && t <= Ts * faai * 2.5)
  {
    sigma = 2 * PI * (t - Ts * faai * 1.5) / (faai * Ts); //0~2*PI
    yep = h * (1 - cos(sigma)) / 2; //0~h~0
    xep_b = 0.75 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~3S/4
    xep_z = -0.25 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~-S/4
    //左后3
    Pedis_y[0] = H+ yep*k_walk;          Pedis_y[1] = H+ yep*k_walk;        Pedis_y[2] = H - yep;       Pedis_y[3] = H+ yep*k_walk;
    Pedis_x[0] = xep_z + S / 2;  Pedis_x[1] = xep_z;    Pedis_x[2] = xep_b - S / 2; Pedis_x[3] = xep_z ;
    Pedis_z[0] = d - wei;      Pedis_z[1] = d + wei;    Pedis_z[2] = d + wei;     Pedis_z[3] = d - wei;

    xep_zs = -0.05 * S + 0.05 * S * ((sigma - sin(sigma)) / (2 * PI)); //-S/20~0
    Pedis_x[0] = Pedis_x[0] + xep_zs;  Pedis_x[1] = Pedis_x[1] + xep_zs; Pedis_x[2] = Pedis_x[2] + xep_zs; Pedis_x[3] = Pedis_x[3] + xep_zs;
  }
  else if (t > Ts * faai * 2.5 && t <= Ts * faai * 3.5)
  {
    sigma = 2 * PI * (t - Ts * faai * 2.5) / (faai * Ts); //0~2*PI
    yep = h * (1 - cos(sigma)) / 2; //0~h~0
    xep_b = 0.75 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~3S/4
    xep_z = -0.25 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~-S/4
    //左前2
    Pedis_y[0] = H+ yep*k_walk;         Pedis_y[1] = H - yep;        Pedis_y[2] = H+ yep*k_walk;         Pedis_y[3] = H+ yep*k_walk;
    Pedis_x[0] = xep_z + S / 4; Pedis_x[1] = xep_b - S / 4;  Pedis_x[2] = xep_z + S / 4; Pedis_x[3] = xep_z - S / 4 ;
    Pedis_z[0] = d - wei;     Pedis_z[1] = d + wei;      Pedis_z[2] = d + wei;     Pedis_z[3] = d - wei;

    xep_zs = 0.05 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~S/20
    Pedis_x[0] = Pedis_x[0] + xep_zs;  Pedis_x[1] = Pedis_x[1] + xep_zs; Pedis_x[2] = Pedis_x[2] + xep_zs; Pedis_x[3] = Pedis_x[3] + xep_zs;
  }
  else if (t > Ts * faai * 3.5 && t <= Ts * faai * 4)
  {
    //四足支撑相
    sigma = 2 * PI * (t - 3.5 * Ts * faai) / (faai * Ts * 0.5); //0~2*PI
    zep = 2 * wei * ((sigma - sin(sigma)) / (2 * PI)); //0~2*wei
    Pedis_y[0] = H;        Pedis_y[1] = H;        Pedis_y[2] = H;        Pedis_y[3] = H;
    Pedis_x[0] = 0;        Pedis_x[1] = S / 2;      Pedis_x[2] = 0;        Pedis_x[3] = -S / 2;
    Pedis_z[0] = d - wei + zep;     Pedis_z[1] = d + wei - zep;    Pedis_z[2] = d + wei - zep;    Pedis_z[3] = d - wei + zep;

    xep_zs = 0.05 * S - 0.1 * S * ((sigma - sin(sigma)) / (2 * PI)); //S/20~-S/20
    Pedis_x[0] = Pedis_x[0] + xep_zs;  Pedis_x[1] = Pedis_x[1] + xep_zs; Pedis_x[2] = Pedis_x[2] + xep_zs; Pedis_x[3] = Pedis_x[3] + xep_zs;
  }
  else if (t > Ts * faai * 4 && t <= Ts * faai * 5)
  {
    sigma = 2 * PI * (t - Ts * faai * 4) / (faai * Ts); //0~2*PI
    yep = h * (1 - cos(sigma)) / 2; //0~h~0
    xep_b = 0.75 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~3S/4
    xep_z = -0.25 * S * ((sigma - sin(sigma)) / (2 * PI)); //0~-S/4
    //右后4
    Pedis_y[0] = H+ yep*k_walk;        Pedis_y[1] = H+ yep*k_walk;         Pedis_y[2] = H+ yep*k_walk;        Pedis_y[3] = H - yep;
    Pedis_x[0] = xep_z;    Pedis_x[1] = xep_z + S / 2; Pedis_x[2] = xep_z;    Pedis_x[3] = xep_b - S / 2 ;
    Pedis_z[0] = d + wei;    Pedis_z[1] = d - wei;     Pedis_z[2] = d - wei;    Pedis_z[3] = d + wei;

    xep_zs = -0.05 * S + 0.05 * S * ((sigma - sin(sigma)) / (2 * PI)); //-S/20~0
    Pedis_x[0] = Pedis_x[0] + xep_zs;  Pedis_x[1] = Pedis_x[1] + xep_zs; Pedis_x[2] = Pedis_x[2] + xep_zs; Pedis_x[3] = Pedis_x[3] + xep_zs;
  }
	Pedis_x[0]=Pedis_x[0]-33;
	Pedis_x[1]=Pedis_x[1]-33;
	Pedis_x[2]=Pedis_x[2]-33;
	Pedis_x[3]=Pedis_x[3]-33;
}




void gait_crab(int t, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
	double sigma = 0.0;
	double zep, xep_b, xep_z;
	int   kkk =0;
	int Zero = -38;
  if (t <= (Ts /2))
  {
		sigma = 4 * PI * t / Ts;
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * PI)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * PI)) + xf;
    //输出y
		Pedis_y[0] = H - zep;
    Pedis_y[1] = H + zep * kkk;
    Pedis_y[2] = H - zep;
    Pedis_y[3] = H + zep * kkk;
    //输出x
		Pedis_x[0] = Zero;
    Pedis_x[1] = Zero;
    Pedis_x[2] = Zero;
    Pedis_x[3] = Zero;
		//printf("Pedis_x:%f ",Pedis_x[0]);
    Pedis_z[0] = d+xep_z;
    Pedis_z[1] = d-xep_b;
    Pedis_z[2] = d-xep_z;
    Pedis_z[3] = d+xep_b;
  }
  else if (t > (Ts / 2) && t <= Ts)
  {
		sigma = 4 * PI * (t - Ts /2 ) / (Ts);
		
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * PI)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * PI)) + xf;
    //输出y
    Pedis_y[0] = H + zep * kkk;
    Pedis_y[1] = H - zep;
    Pedis_y[2] = H + zep * kkk;
    Pedis_y[3] = H - zep;
    //输出x
    Pedis_x[0] = Zero;
    Pedis_x[1] = Zero;
    Pedis_x[2] = Zero;
    Pedis_x[3] = Zero;
    Pedis_z[0] = d+xep_b;
    Pedis_z[1] = d-xep_z;
    Pedis_z[2] = d-xep_b;
    Pedis_z[3] = d+xep_z;
  }
  
  else {
    Pedis_x[0] = 0;
		Pedis_x[1] = 0;
		Pedis_x[2] = 0;
		Pedis_x[3] = 0;

		Pedis_y[0] = H;
		Pedis_y[1] = H;
		Pedis_y[2] = H;
		Pedis_y[3] = H;
  }
	return;
}



void gait_round(int t, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
	double sigma = 0.0;
	double zep, xep_b, xep_z;
	int   kkk =0;
	int Zero = -38;
  if (t <= (Ts /2))
  {
		sigma = 4 * PI * t / Ts;
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * PI)) + xs; //xs-xf
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * PI)) + xf;//xf-xs
    //输出y
		Pedis_y[0] = H - zep;
    Pedis_y[1] = H + zep * kkk;
    Pedis_y[2] = H - zep;
    Pedis_y[3] = H + zep * kkk;
		//输出x
		Pedis_x[0] = Zero;
    Pedis_x[1] = Zero;
    Pedis_x[2] = Zero;
    Pedis_x[3] = Zero;
		//printf("Pedis_x:%f ",Pedis_x[0]);
    Pedis_z[0] = d+xep_z;
    Pedis_z[1] = d-xep_b;
    Pedis_z[2] = d+xep_z;
    Pedis_z[3] = d-xep_b;
		
  }
	else if (t > (Ts / 2) && t <= Ts)
  {
		sigma = 4 * PI * (t - Ts /2 ) / (Ts);
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * PI)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * PI)) + xf;
    //输出y
		Pedis_y[0] = H + zep * kkk;
    Pedis_y[1] = H - zep;
    Pedis_y[2] = H + zep * kkk;
    Pedis_y[3] = H - zep;
		//输出x
		Pedis_x[0] = Zero;
    Pedis_x[1] = Zero;
    Pedis_x[2] = Zero;
    Pedis_x[3] = Zero;
		//printf("Pedis_x:%f ",Pedis_x[0]);
    Pedis_z[0] = d+xep_b;
    Pedis_z[1] = d-xep_z;
    Pedis_z[2] = d+xep_b;
    Pedis_z[3] = d-xep_z;
  }
 
  else {
		Pedis_x[0] = Zero;
    Pedis_x[1] = Zero;
    Pedis_x[2] = Zero;
    Pedis_x[3] = Zero;

		Pedis_y[0] = H;
		Pedis_y[1] = H;
		Pedis_y[2] = H;
		Pedis_y[3] = H;
  }
}




void gait_step(int t, double h, double zero_step)
{
	double kkk =0;
	double sigma = 0.0;
	
	double zep, xep_b, xep_z;

  if (t <= (Ts /2))
  {
		//printf("t <= Ts /2\r\n");
    sigma = 4 * PI * t / Ts; 
    zep = h * (1 - cos(sigma)) / 2;
    //输出y
    Pedis_y[0] = H - zep;
    Pedis_y[1] = H + zep * kkk;
    Pedis_y[2] = H - zep;
    Pedis_y[3] = H + zep * kkk;
		//printf("Pedis_y:%f ",Pedis_y[0]);
    //输出x
    Pedis_x[0] = zero_step;
    Pedis_x[1] = zero_step;
    Pedis_x[2] = zero_step;
    Pedis_x[3] = zero_step;
		//printf("Pedis_x:%f ",Pedis_x[0]);
    Pedis_z[0] = d;
    Pedis_z[1] = d;
    Pedis_z[2] = d;
    Pedis_z[3] = d;
		//printf("Pedis_z:%f \r\n",Pedis_z[0]);
  }
  else if (t > (Ts / 2) && t <= Ts)
  {
    sigma = 4 * PI * (t - Ts /2 ) / (Ts);
		
    zep = h * (1 - cos(sigma)) / 2;
    //输出y
    Pedis_y[0] = H + zep* kkk;
    Pedis_y[1] = H - zep;
    Pedis_y[2] = H + zep* kkk;
    Pedis_y[3] = H - zep;
    //输出x
    Pedis_x[0] = zero_step;
    Pedis_x[1] = zero_step;
    Pedis_x[2] = zero_step;
    Pedis_x[3] = zero_step;
    Pedis_z[0] = d;
    Pedis_z[1] = d;
    Pedis_z[2] = d;
    Pedis_z[3] = d;

  }
  else {
    Pedis_x[0] = zero_step;
    Pedis_x[1] = zero_step;
    Pedis_x[2] = zero_step;
    Pedis_x[3] = zero_step;
		
		Pedis_y[0] = H;
		Pedis_y[1] = H;
		Pedis_y[2] = H;
		Pedis_y[3] = H;
  }
	return;
}



void balance(double PItch2, double roll2, double yaw)
{
  double PItch = PId_algo(&PItch_PId,PItch2);
  double roll = PId_algo(&roll_PId,roll2);

  PItch = -PItch * PI / 180; roll = roll * PI / 180; yaw = yaw * PI / 180;

  double LL = 200; double ww = 210; 
	
  double  dx1 = LL / 2, dy1 = -H, dz1 = -ww / 2;
  Pedis_x[3] = -(cos(PItch) * cos(yaw) * dx1 - sin(PItch) * dy1 + cos(PItch) * sin(yaw) * dz1 - LL / 2);
  Pedis_y[3] =  -(dx1 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(PItch)) - dz1 * (cos(roll) * sin(roll) - cos(roll) * sin(PItch) * sin(yaw)) + cos(PItch) * cos(roll) * dy1);
  Pedis_z[3] = -(dz1 * (cos(roll) * cos(yaw) + sin(PItch) * sin(roll) * sin(yaw)) - dx1 * (cos(roll) * sin(yaw) - cos(yaw) * sin(PItch) * sin(roll)) + cos(PItch) * sin(roll) * dy1 + ww / 2) + 38;

  double dx2 = LL / 2, dy2 = -H, dz2 = ww / 2;
  Pedis_x[2] = -(cos(PItch) * cos(yaw) * dx2 - sin(PItch) * dy2 + cos(PItch) * sin(yaw) * dz2 - LL / 2);
  Pedis_y[2] = -(dx2 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(PItch)) - dz2 * (cos(roll) * sin(roll) - cos(roll) * sin(PItch) * sin(yaw)) + cos(PItch) * cos(roll) * dy2);
  Pedis_z[2] = (dz2 * (cos(roll) * cos(yaw) + sin(PItch) * sin(roll) * sin(yaw)) - dx2 * (cos(roll) * sin(yaw) - cos(yaw) * sin(PItch) * sin(roll)) + cos(PItch) * sin(roll) * dy2 - ww / 2) + 38;

  double dx3 = -LL / 2, dy3 = -H, dz3 = ww / 2;
  Pedis_x[1] = -(cos(PItch) * cos(yaw) * dx3 - sin(PItch) * dy3 + cos(PItch) * sin(yaw) * dz3 + LL / 2);
  Pedis_y[1] = -(dx3 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(PItch)) - dz3 * (cos(roll) * sin(roll) - cos(roll) * sin(PItch) * sin(yaw)) + cos(PItch) * cos(roll) * dy3);
  Pedis_z[1] = (dz3 * (cos(roll) * cos(yaw) + sin(PItch) * sin(roll) * sin(yaw)) - dx3 * (cos(roll) * sin(yaw) - cos(yaw) * sin(PItch) * sin(roll)) + cos(PItch) * sin(roll) * dy3 - ww / 2) + 38;

  double  dx4 = -LL / 2, dy4 = -H, dz4 = -ww / 2;
  Pedis_x[0] = -(cos(PItch) * cos(yaw) * dx4 - sin(PItch) * dy4 + cos(PItch) * sin(yaw) * dz4 + LL / 2);
  Pedis_y[0] = -(dx4 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(PItch)) - dz4 * (cos(roll) * sin(roll) - cos(roll) * sin(PItch) * sin(yaw)) + cos(PItch) * cos(roll) * dy4);
  Pedis_z[0] = -(dz4 * (cos(roll) * cos(yaw) + sin(PItch) * sin(roll) * sin(yaw)) - dx4 * (cos(roll) * sin(yaw) - cos(yaw) * sin(PItch) * sin(roll)) + cos(PItch) * sin(roll) * dy4 + ww / 2) + 38;

}


void Trot_state(double sp, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
	LSCControlPeriod = sp;
	gait_trot(t, xs, xf, h, r1 ,r2 ,r3 ,r4);
	if(++t >= Ts)t=0;
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}


void Walk_state(double sp, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
	LSCControlPeriod = sp;
	gait_walk(t, xs, xf, h, r1 ,r2 ,r3 ,r4);
	if(++t >= Ts)t=0;
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}


void Crab_state(double sp, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
	LSCControlPeriod = sp;
	gait_crab(t, xs, xf, h, r1 ,r2 ,r3 ,r4);
	if(++t >= Ts)t=0;
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}


void Round_state(double sp, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
	LSCControlPeriod = sp;
	gait_round(t, xs, xf, h, r1 ,r2 ,r3 ,r4);
	if(++t >= Ts)t=0;
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}


void Step_state(double sp, double h, double zero_step)
{
	LSCControlPeriod = sp;
	gait_step(t, h, zero_step);
	if(++t >= Ts)t=0;
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}

void Balance_state(double sp)
{
	//OLED_ShowStr(0, 1, "State: Balance", 1);
	//usb_printf("Balance_run\r\n");
	LSCControlPeriod = sp;
	JY901_RDDat(&IMU);
	balance(IMU.Angle[0], IMU.Angle[1], 0);
	if(++t >= Ts)t=0;
  ik_Move(Pedis_x, Pedis_y, Pedis_z);
//	OLED_ShowStr(0, 1, "State: Balance", 1);
//	usb_printf("Balance_run\r\n");
//	JY901_RDDat(&IMU);
//	sprintf((char*)str,"a:%.1f %.1f %.1f",IMU.a[0],IMU.a[1],IMU.a[2]);
//	OLED_ShowStr(6, 2, str, 1);
//	sprintf((char*)str,"w:%.1f %.1f %.1f",IMU.w[0],IMU.w[1],IMU.w[2]);
//	OLED_ShowStr(6, 3, str, 1);
//	sprintf((char*)str,"h:%.1f %.1f %.1f",IMU.h[0],IMU.h[1],IMU.h[2]);
//	OLED_ShowStr(6, 4, str, 1);
//	sprintf((char*)str,"A:%.1f %.1f %.1f",IMU.Angle[0],IMU.Angle[1],IMU.Angle[2]);
//	OLED_ShowStr(6, 5, str, 1);
}

void Trot_run(void)
{
  Trot_state(50, 8, -60, 50, 1, 1, 1, 1);
}

void Walk_run(void)
{
	Walk_state(150, 0, 85,60, 1,1,1,1);
}



void Stand_run(void)
{
	LSCControlPeriod = 500;
	gait_Init();
	
	osDelay(10);
}
void Dance_run(void)
{
}
void Bend_run(void)
{
	osDelay(10);
}

void Balance_run(void)
{
	Balance_state(50);
}

void Walk_Bend_run(void)
{
	osDelay(10);
}

void Crab_L_run(void)
{
  Crab_state(50, 15, -15, 40, 1, 1, 1, 1);
}

void Crab_R_run(void)
{
	Crab_state(50, -15, 15, 40, 1, 1, 1, 1);
}

void Round_L_run(void)
{
  Round_state(60, 0, -40, 40, 1, 1, 1, 1);
}

void Round_R_run(void)
{
	Round_state(60, 0, 40, 40, 1, 1, 1, 1);
}

void Step_run(void)
{
	Step_state(50,40,-30);
}

void Stand_Up_run(void)
{
//	if (t < Ts_s) t += sspeed;
//  stand_up(t);
//  ik_3dof(l2, l3, x1, x2, x3, x4, y1, y2, y3, y4, z1, z2, z3, z4);
//  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8], G[9], G[10], G[11]);
	osDelay(10);
}

void Power_down(void)
{
	osDelay(10);
}

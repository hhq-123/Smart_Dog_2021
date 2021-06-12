#include "test_workspace.h"
#include "Quadrudep_huaner.h"
#include "usbd_cdc_if.h"
#include "stdio.h"
#include "math.h"
#include "JY901.h"
#include "OLED.h"

extern JY901_Angle IMU;
unsigned char str[100];

double Pedis_x[4];
double Pedis_y[4];
double Pedis_z[4];

int Ts = 6;

int t = 0;

void gait_trot(double t, double xs, double xf, double h, double r1, double r4, double r2, double r3) 
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

void Trot_run(void)
{
	OLED_ShowStr(0, 1, "State: Trot", 1);
	usb_printf("Trot_run\r\n");
  gait_trot(t, -23, -60, 70, 1, 1, 1, 1);
	if(++t == Ts)t=0;
	//printf("Pedis_y%f\r\n", Pedis_y[0]);
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
	//printf("finish\r\n");
	for(int i =0; i<4; i++)
	{
		sprintf((char*)str,"l:%.1f %.1f %.1f",Pedis_x[i],Pedis_y[i],Pedis_z[i]);
		OLED_ShowStr(6, 2+i, str, 1);
	}
}

void Walk_run(void)
{
	OLED_ShowStr(0, 1, "State: Walk", 1);
	usb_printf("Walk_run\r\n");
}

void Balance_run(void)
{
	OLED_ShowStr(0, 1, "State: Balance", 1);
	usb_printf("Balance_run\r\n");
	JY901_RDDat(&IMU);
	sprintf((char*)str,"a:%.1f %.1f %.1f",IMU.a[0],IMU.a[1],IMU.a[2]);
	OLED_ShowStr(6, 2, str, 1);
	sprintf((char*)str,"w:%.1f %.1f %.1f",IMU.w[0],IMU.w[1],IMU.w[2]);
	OLED_ShowStr(6, 3, str, 1);
	sprintf((char*)str,"h:%.1f %.1f %.1f",IMU.h[0],IMU.h[1],IMU.h[2]);
	OLED_ShowStr(6, 4, str, 1);
	sprintf((char*)str,"A:%.1f %.1f %.1f",IMU.Angle[0],IMU.Angle[1],IMU.Angle[2]);
	OLED_ShowStr(6, 5, str, 1);
}



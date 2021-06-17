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

int Ts = 10;

int t = 0;

void gait_trot(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3) 
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
	int wei=5;
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
	Pedis_x[0]=Pedis_x[0]-30;
	Pedis_x[1]=Pedis_x[1]-30;
	Pedis_x[2]=Pedis_x[2]-30;
	Pedis_x[3]=Pedis_x[3]-30;
}


void gait_crab(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3)
{
	double sigma = 0.0;
	double zep, xep_b, xep_z;
	int   kkk =0;
	int Zero = -40;
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

void gait_round(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3)
{
	double sigma = 0.0;
	double zep, xep_b, xep_z;
	int   kkk =0;
	int Zero = -35;
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
    Pedis_x[0] = 0;
		Pedis_x[1] = 0;
		Pedis_x[2] = 0;
		Pedis_x[3] = 0;

		Pedis_y[0] = H;
		Pedis_y[1] = H;
		Pedis_y[2] = H;
		Pedis_y[3] = H;
  }
}

void Trot_run(void)
{
//	OLED_ShowStr(0, 1, "State: Trot", 1);
//	usb_printf("Trot_run\r\n");
  gait_trot(t, 0, -70, 50, 1, 1, 1, 1);
	if(++t == Ts)t=0;
//	printf("Pedis_y%f\r\n", Pedis_y[0]);
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
//	printf("finish\r\n");
//	for(int i =0; i<4; i++)
//	{
//		sprintf((char*)str,"l:%.1f %.1f %.1f",Pedis_x[i],Pedis_y[i],Pedis_z[i]);
//		OLED_ShowStr(6, 2+i, str, 1);
//	}
}

void Walk_run(void)
{
	gait_walk(t, 0, 60,60, 1,1,1,1);
	if(++t == Ts)t=0;
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
	OLED_ShowStr(0, 1, "State: Walk", 1);
	usb_printf("Walk_run\r\n");
}

void Stand_run(void)
{
}
void Dance_run(void)
{
}
void Bend_run(void)
{
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
void Walk_Bend_run(void)
{
	
}
void Crab_L_run(void)
{
  gait_crab(t, 15, -15, 40, 1, 1, 1, 1);
	if(++t == Ts)t=0;
	//printf("Pedis_y%f\r\n", Pedis_y[0]);
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}

void Crab_R_run(void)
{
	gait_crab(t, -15, 15, 40, 1, 1, 1, 1);
	if(++t == Ts)t=0;
	//printf("Pedis_y%f\r\n", Pedis_y[0]);
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}

void Round_L_run(void)
{
  gait_round(t, 15, -25, 50, 1, 1, 1, 1);
	if(++t == Ts)t=0;
	//printf("Pedis_y%f\r\n", Pedis_y[0]);
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}

void Round_R_run(void)
{
	gait_round(t, -25, 15, 50, 1, 1, 1, 1);
	if(++t == Ts)t=0;
	//printf("Pedis_y%f\r\n", Pedis_y[0]);
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}

void Stand_Up_run(void)
{
//	if (t < Ts_s) t += sspeed;
//  stand_up(t);
//  ik_3dof(l2, l3, x1, x2, x3, x4, y1, y2, y3, y4, z1, z2, z3, z4);
//  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8], G[9], G[10], G[11]);
}

void Power_down(void)
{

}

#define pi 3.1415
#include <Wire.h>
#include <JY901.h>
SoftwareSerial mine(2, 3);
#include "Quadrudep_huaner.h"

double x, y;

double shank1, shank2, shank3, shank4, ham1, ham2, ham3, ham4, wai1, wai2, wai3, wai4;
double G[12];
double faai_walk = 0.2; double Ts_walk = 2;
double xs = 0, xf = 0;
double yaw = 0, pitch = 0, roll = 0;
double wei = 0;

typedef struct {
  double kp, ki, kd;
  double error, output,last_error, integral;
} pid_struct_typedef;



//typedef struct{
//    double error,output = 0,kp = 0.2,ki = 0, kd = 0, last_error = 0,integral = 0;
//}pid_struct_typedef;

pid_struct_typedef pitch_pid = {1.0,0,0.2,   0,0,0};

pid_struct_typedef roll_pid = {1.0,0,0.2,   0,0,0};


//double kp = 0.2, ki = 0, kd = 0;
//  double error, output = 0,last_error = 0, integral = 0;

double pid_algo(pid_struct_typedef *pid, double fdb) {
  pid->error = fdb;

  pid->output = pid->error * pid->kp + pid->integral * pid->ki + (pid->error - pid->last_error) * pid->kd;

  pid->integral += pid->error;
  pid->last_error = pid->error;

  if (pid->integral > 20) pid->integral = 20;
  else if (pid->integral < -20) pid->integral = -20;

  return pid->output;
}




void balance(double pitch2, double roll2, double yaw)
{
  pitch = pid_algo(&pitch_pid,pitch2);
  roll = pid_algo(&roll_pid,roll2);

  //          pitch = pitch+k*pitch2;roll = roll +k*roll2;
  //          if (pitch>20)pitch=20;
  //          // if (pitch<-20)pitch=-20;
  //           if (roll>20)roll=20;
  //           // if (roll<-20)roll=-20;
  pitch = -pitch * pi / 180; roll = roll * pi / 180; yaw = yaw * pi / 180;

  double LL = 200; double ww = 210; H = 180;
  double  dx1 = LL / 2, dy1 = -H, dz1 = -ww / 2;
  x4 = -(cos(pitch) * cos(yaw) * dx1 - sin(pitch) * dy1 + cos(pitch) * sin(yaw) * dz1 - LL / 2);
  y4 =  -(dx1 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - dz1 * (cos(roll) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + cos(pitch) * cos(roll) * dy1);
  z4 = -(dz1 * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - dx1 * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(pitch) * sin(roll) * dy1 + ww / 2) + 38;

  double dx2 = LL / 2, dy2 = -H, dz2 = ww / 2;
  x3 = -(cos(pitch) * cos(yaw) * dx2 - sin(pitch) * dy2 + cos(pitch) * sin(yaw) * dz2 - LL / 2);
  y3 = -(dx2 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - dz2 * (cos(roll) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + cos(pitch) * cos(roll) * dy2);
  z3 = (dz2 * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - dx2 * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(pitch) * sin(roll) * dy2 - ww / 2) + 38;

  double dx3 = -LL / 2, dy3 = -H, dz3 = ww / 2;
  x2 = -(cos(pitch) * cos(yaw) * dx3 - sin(pitch) * dy3 + cos(pitch) * sin(yaw) * dz3 + LL / 2);
  y2 = -(dx3 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - dz3 * (cos(roll) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + cos(pitch) * cos(roll) * dy3);
  z2 = (dz3 * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - dx3 * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(pitch) * sin(roll) * dy3 - ww / 2) + 38;

  double  dx4 = -LL / 2, dy4 = -H, dz4 = -ww / 2;
  x1 = -(cos(pitch) * cos(yaw) * dx4 - sin(pitch) * dy4 + cos(pitch) * sin(yaw) * dz4 + LL / 2);
  y1 = -(dx4 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - dz4 * (cos(roll) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + cos(pitch) * cos(roll) * dy4);
  z1 = -(dz4 * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - dx4 * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(pitch) * sin(roll) * dy4 + ww / 2) + 38;

}

void gait_walk(double t, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
   double faai = 0.2;
  double k_walk=0.2;
  //前面加上定义yep(抬腿）,wei（侧摆）,ww(为机器人的宽)，xep-zs支撑相的移动
  S = 4 * (xf - xs) / 3; //S为步长，相对地面的步长是相对身体步长的4/3

  
  double yep, xep_zs;
  
  if (t <= Ts_walk * faai) //faai=0.2
  {
    sigma = 2 * pi * t / (faai * Ts_walk); //0~2*pi
    yep = h * (1 - cos(sigma)) / 2; //0~h~0
    xep_b = 0.75 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~3S/4
    xep_z = -0.25 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~-S/4
    //右前1
    y1 = H - yep;        y2 = H+ yep*k_walk;         y3 = H+ yep*k_walk;         y4 = H+ yep*k_walk;
    x1 = xep_b - S / 4;  x2 = xep_z + S / 4; x3 = xep_z - S/4; x4 = xep_z + S/4 ;
    z1 = d + wei;      z2 = d - wei;     z3 = d - wei;     z4 = d + wei;

    xep_zs = 0.05 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~S/20
    x1 = x1 + xep_zs;  x2 = x2 + xep_zs; x3 = x3 + xep_zs; x4 = x4 + xep_zs;
  }
  else if (t > Ts_walk * faai && t <= Ts_walk * faai * 1.5)
  {
    //四足支撑相
    sigma = 2 * pi * (t - Ts_walk * faai) / (faai * Ts_walk * 0.5); //0~2*pi
    zep = 2 * wei * ((sigma - sin(sigma)) / (2 * pi)); //0~2*wei
    y1 = H;        y2 = H;        y3 = H;        y4 = H;
    x1 = S / 2;      x2 = 0;        x3 = -S / 2;     x4 = 0;
    z1 = d + wei - zep;     z2 = d - wei + zep;    z3 = d - wei + zep;    z4 = d + wei - zep;

    xep_zs = 0.05 * S - 0.1 * S * ((sigma - sin(sigma)) / (2 * pi)); //S/20~-S/20
    x1 = x1 + xep_zs;  x2 = x2 + xep_zs; x3 = x3 + xep_zs; x4 = x4 + xep_zs;
  }
  else if (t > Ts_walk * faai * 1.5 && t <= Ts_walk * faai * 2.5)
  {
    sigma = 2 * pi * (t - Ts_walk * faai * 1.5) / (faai * Ts_walk); //0~2*Pi
    yep = h * (1 - cos(sigma)) / 2; //0~h~0
    xep_b = 0.75 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~3S/4
    xep_z = -0.25 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~-S/4
    //左后3
    y1 = H+ yep*k_walk;          y2 = H+ yep*k_walk;        y3 = H - yep;       y4 = H+ yep*k_walk;
    x1 = xep_z + S / 2;  x2 = xep_z;    x3 = xep_b - S / 2; x4 = xep_z ;
    z1 = d - wei;      z2 = d + wei;    z3 = d + wei;     z4 = d - wei;

    xep_zs = -0.05 * S + 0.05 * S * ((sigma - sin(sigma)) / (2 * pi)); //-S/20~0
    x1 = x1 + xep_zs;  x2 = x2 + xep_zs; x3 = x3 + xep_zs; x4 = x4 + xep_zs;
  }
  else if (t > Ts_walk * faai * 2.5 && t <= Ts_walk * faai * 3.5)
  {
    sigma = 2 * pi * (t - Ts_walk * faai * 2.5) / (faai * Ts_walk); //0~2*Pi
    yep = h * (1 - cos(sigma)) / 2; //0~h~0
    xep_b = 0.75 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~3S/4
    xep_z = -0.25 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~-S/4
    //左前2
    y1 = H+ yep*k_walk;         y2 = H - yep;        y3 = H+ yep*k_walk;         y4 = H+ yep*k_walk;
    x1 = xep_z + S / 4; x2 = xep_b - S / 4;  x3 = xep_z + S / 4; x4 = xep_z - S / 4 ;
    z1 = d - wei;     z2 = d + wei;      z3 = d + wei;     z4 = d - wei;

    xep_zs = 0.05 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~S/20
    x1 = x1 + xep_zs;  x2 = x2 + xep_zs; x3 = x3 + xep_zs; x4 = x4 + xep_zs;
  }
  else if (t > Ts_walk * faai * 3.5 && t <= Ts_walk * faai * 4)
  {
    //四足支撑相
    sigma = 2 * pi * (t - 3.5 * Ts_walk * faai) / (faai * Ts_walk * 0.5); //0~2*pi
    zep = 2 * wei * ((sigma - sin(sigma)) / (2 * pi)); //0~2*wei
    y1 = H;        y2 = H;        y3 = H;        y4 = H;
    x1 = 0;        x2 = S / 2;      x3 = 0;        x4 = -S / 2;
    z1 = d - wei + zep;     z2 = d + wei - zep;    z3 = d + wei - zep;    z4 = d - wei + zep;

    xep_zs = 0.05 * S - 0.1 * S * ((sigma - sin(sigma)) / (2 * pi)); //S/20~-S/20
    x1 = x1 + xep_zs;  x2 = x2 + xep_zs; x3 = x3 + xep_zs; x4 = x4 + xep_zs;
  }
  else if (t > Ts_walk * faai * 4 && t <= Ts_walk * faai * 5)
  {
    sigma = 2 * pi * (t - Ts_walk * faai * 4) / (faai * Ts_walk); //0~2*Pi
    yep = h * (1 - cos(sigma)) / 2; //0~h~0
    xep_b = 0.75 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~3S/4
    xep_z = -0.25 * S * ((sigma - sin(sigma)) / (2 * pi)); //0~-S/4
    //右后4
    y1 = H+ yep*k_walk;        y2 = H+ yep*k_walk;         y3 = H+ yep*k_walk;        y4 = H - yep;
    x1 = xep_z;    x2 = xep_z + S / 2; x3 = xep_z;    x4 = xep_b - S / 2 ;
    z1 = d + wei;    z2 = d - wei;     z3 = d - wei;    z4 = d + wei;

    xep_zs = -0.05 * S + 0.05 * S * ((sigma - sin(sigma)) / (2 * pi)); //-S/20~0
    x1 = x1 + xep_zs;  x2 = x2 + xep_zs; x3 = x3 + xep_zs; x4 = x4 + xep_zs;
  }

}
/*---------------------------walk---end--------------------*/


void gait_walk1(double t, double xs, double xf, double h, double r1, double r2, double r3, double r4)
{
  S = 4 * (xf - xs) / 3; //S为步长，相对地面的步长是相对身体步长的4/3
  if (t <= Ts_walk * faai_walk) //faai=0.2
  {
    sigma = 2 * pi * t / (faai_walk * Ts_walk);
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = S * ((sigma - sin(sigma)) / (2 * pi));
    //右前
    //输出y
    y1 = H - zep;
    y2 = H;
    y3 = H;
    y4 = H;
    //输出x
    x1 = xep_b - S / 4;
    x2 = S / 4;
    x3 = -S / 4;
    x4 = S / 4  ;
  }
  else if (t > Ts_walk * faai_walk && t <= Ts_walk * faai_walk * 1.5)
  {
    //四足支撑相
    sigma = 2 * pi * (t - Ts_walk * faai_walk) / (faai_walk * Ts_walk * 0.5);
    xep_z = -0.5 * S * ((sigma - sin(sigma)) / (2 * pi));
    //输出y
    y1 = H;
    y2 = H;
    y3 = H;
    y4 = H;
    //输出x
    x1 = xep_z + 0.75 * S;
    x2 = xep_z + 0.25 * S;
    x3 = xep_z - 0.25 * S;
    x4 = xep_z + 0.25 * S;
  }
  else if (t > Ts_walk * faai_walk * 1.5 && t <= Ts_walk * faai_walk * 2.5)
  {
    sigma = 2 * pi * (t - Ts_walk * faai_walk * 1.5) / (faai_walk * Ts_walk);
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = S * ((sigma - sin(sigma)) / (2 * pi));
    //左后
    //输出y
    y1 = H;
    y2 = H;
    y3 = H - zep;
    y4 = H;
    //输出x
    x1 = S / 4;
    x2 = -S / 4;
    x3 = xep_b - 3 * S / 4;
    x4 = -S / 4;

  }
  else if (t > Ts_walk * faai_walk * 2.5 && t <= Ts_walk * faai_walk * 3.5)
  {
    sigma = 2 * pi * (t - Ts_walk * faai_walk * 2.5) / (faai_walk * Ts_walk);
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = S * ((sigma - sin(sigma)) / (2 * pi));

    //左前
    //输出y
    y1 = H;
    y2 = H - zep;
    y3 = H;
    y4 = H;
    //输出x
    x1 = S / 4;
    x2 = xep_b - S / 4;
    x3 = S / 4;
    x4 = -S / 4;
  }
  else if (t > Ts_walk * faai_walk * 3.5 && t <= Ts_walk * faai_walk * 4)
  {
    sigma = 2 * pi * (t - Ts_walk * faai_walk * 3.5) / (faai_walk * Ts_walk * 0.5);
    xep_z = -0.5 * S * ((sigma - sin(sigma)) / (2 * pi));
    //四足支撑相
    //输出y
    y1 = H;
    y2 = H;
    y3 = H;
    y4 = H;
    //输出x
    x1 = xep_z + 0.25 * S;
    x2 = xep_z + 0.75 * S;
    x3 = xep_z + 0.25 * S;
    x4 = xep_z - 0.25 * S;
  }
  else if (t > Ts_walk * faai_walk * 4 && t <= Ts_walk * faai_walk * 5)
  {
    sigma = 2 * pi * (t - Ts_walk * faai_walk * 4) / (faai_walk * Ts_walk);
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = S * ((sigma - sin(sigma)) / (2 * pi));

    //右后
    //输出y
    y1 = H;
    y2 = H;
    y3 = H;
    y4 = H - zep;
    //输出x
    x1 = -S / 4;
    x2 = S / 4;
    x3 = -S / 4;
    x4 = xep_b - 3 * S / 4;
  }
  z1 = d;
  z2 = d;
  z3 = d;
  z4 = d;
}


void gait_trot(double t, double xs, double xf, double h, double r1, double r4, double r2, double r3) {
  int   kkk =0.8, m = 5; faai = 0.5;
  if (t <= Ts * faai)
  {
    sigma = 2 * pi * t / (faai * Ts);
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * pi)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * pi)) + xf;
    //输出y
    y1 = H - zep;
    y2 = H + zep * kkk;
    y3 = H - zep;
    y4 = H + zep * kkk;
    //输出x
    x1 = xep_z * r1;
    x2 = xep_b * r2;
    x3 = xep_z * r3;
    x4 = xep_b * r4;
    z1 = d;
    z2 = d;
    z3 = d;
    z4 = d;
  }
  else if (t > Ts * faai && t <= Ts)
  {
    sigma = 2 * pi * (t - Ts * faai) / (faai * Ts);
    zep = h * (1 - cos(sigma)) / 2;
    xep_b = (xf - xs) * ((sigma - sin(sigma)) / (2 * pi)) + xs;
    xep_z = (xs - xf) * ((sigma - sin(sigma)) / (2 * pi)) + xf;
    //输出y
    y1 = H + zep * kkk;
    y2 = H - zep;
    y3 = H + zep * kkk;
    y4 = H - zep;
    //输出x
    x1 = xep_b * r1;
    x2 = xep_z * r2;
    x3 = xep_b * r3;
    x4 = xep_z * r4;
    z1 = d;
    z2 = d;
    z3 = d;
    z4 = d;

  }
  else {
    x1 = 0; x2 = 0; x3 = 0; x4 = 0;
    y1 = H; y2 = H; y3 = H; y4 = H;
  }
}
double Ts_s = 0.8;
void stand_up(double t)
{


  if (t <= Ts_s) {
    int xep, y;
    sigma = 2 * pi * (t - Ts_s) / (Ts_s);
    xep = 60 * ((sigma - sin(sigma)) / (2 * pi));
    y = 180 + xep;
    y1 = y; y2 = y; y3 = y; y4 = y;
  }
  else {
    y1 = H;
    y2 = H;
    y3 = H;
    y4 = H;
  }
  x1 = -20;
  x2 = -20;
  x3 = -20;
  x4 = -20;
  z1 = d;
  z2 = d;
  z3 = d;
  z4 = d;
}

void jump(double t)
{
double H_jump=210;
if(t <=  Ts_s)
   {
    int xep, y;
    sigma = 2 * pi * (t - Ts_s) / (Ts_s);
    xep = -70 * ((sigma - sin(sigma)) / (2 * pi));
    y = 100 + xep;
    y1 = y; y2 = y; y3 = y; y4 = y;
    x1 = -60;
    x2 = -60;
    x3 = -80;
    x4 = -80;
    z2 = d;
    z3 = d;
    z4 = d;
    
  }
  else if( t <= Ts_s+6*sspeed){
    y1 = H_jump;
    y2 = H_jump;
    y3 = H_jump;
    y4 = H_jump;
  
    x1 = -80;
    x2 = -80;
    x3 = -100;
    x4 = -100;
    z1 = d;
    z2 = d;
    z3 = d;
    z4 = d;
  }
  else {
    
    y1 = 110;
    y2 = 110;
    y3 = 110;
    y4 = 110;
    x1 = -40;
    x2 = -40;
    x3 = -60;
    x4 = -60;
    z1 = d;
    z2 = d;
    z3 = d;
    z4 = d;
    
  }
  
}

void Trot_run()
{
  xs = -23; xf = -60; hh = 70; Ts = 0.15; sspeed = 0.015; H = 180; 
  if (t >= Ts - sspeed) t = 0;
  else t = t + sspeed;
  gait_trot(t, xs, xf, hh, r1, r2, r3, r4);
	ik_Move(Pedis_x, Pedis_y, Pedis_z);
}

int xs_walk, xf_walk;
void Walk_run()
{
  double A = 60;
  wei = 18;
  xs_walk = 0; xf_walk = 50; Ts_walk = 0.6;H=180;
  if (t >= Ts_walk - sspeed) t = 0;
  else t = t + sspeed;
  gait_walk(t, xs_walk, xf_walk, 60, 1, 1, 1, 1);
  ik_3dof(l2, l3, x1 - A, x2 - A, x3 - A, x4 - A, y1, y2, y3, y4, z1, z2, z3, z4);
  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7],G[8], G[9], G[10], G[11]);// 0,0,0,0);//
}

void Walk_bend_run()
{
  double A = 20;
   wei = 10;
  xs_walk = 0; xf_walk = 40; Ts_walk = 1.0  ;H=120;
  if (t >= Ts_walk - sspeed) t = 0;
  else t = t + sspeed;
  gait_walk(t, xs_walk, xf_walk, 60, 1, 1, 1, 1);
  ik_3dof(l2, l3, x1 - A, x2 - A, x3 - A, x4 - A, y1, y2, y3, y4, z1, z2, z3, z4);
  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7],G[8], G[9], G[10], G[11]);// 0,0,0,0);//
}

void Balance_run()
{
  JY901.GetAngle();
  double roll_data = -(float)JY901.stcAngle.Angle[0] / 32768 * 180;
  double pitch_data = (float)JY901.stcAngle.Angle[1] / 32768 * 180;
  //yaw = (float)JY901.stcAngle.Angle[2]/32768*180;
  balance(pitch_data, roll_data, 0);
  int A=30;
  ik_3dof(l2, l3, x1 - A, x2 - A, x3 - A, x4 - A, y1, y2, y3, y4, z1+20, z2+20, z3+20, z4+20);
  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8], G[9], G[10], G[11]);
}

void stand_run()
{
  int X = -20, H = 100 , Z  = d ;
  ik_3dof(l2, l3, X, X, X, X, H, H, H, H, Z, Z, Z, Z);
  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8], G[9], G[10], G[11]);
}
void up()
{
  int i;
  for (i = 0; i < 16; i++)
  { setServo(2, 95);
    delay(500);
    setServo(2, 150);
    delay(500);
  }
}

void bend_run()
{
  int X = -10, H = 110 , Z  = d ;
  ik_3dof(l2, l3, X, X, X, X, H, H, H, H, Z, Z, Z, Z);
  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8], G[9], G[10], G[11]);
}
double i=0;double Tss=8;
void dance_run()
{
  int D=6;
  if(i>Tss)i=0;
  else i+=0.025;
  double faii=0.25;
 if (0<=i && i<Tss*faii){
 yaw=D*sin(2*pi*i/(0.25*Tss*faii))*pi/180;
    pitch=0;
    roll=0;
}
  
else if (Tss*faii<=i && i<Tss*faii*2){
 pitch=D*sin(2*pi*(i-Tss*faii)/(0.25*Tss*faii))*pi/180;
    yaw=0;
    roll=0;
}
       
else if (Tss*faii*2<=i && i<Tss*faii*3){
 pitch=D*sin(2*pi*(i-Tss*faii*3)/(0.25*Tss*faii))*pi/180;
    yaw=D*sin(2*pi*(i-Tss*faii*3)/(0.5*Tss*faii))*pi/180;
    roll=0;
}
else if (Tss*faii*3<=i && i<Tss*faii*4){
 pitch=D*sin(2*pi*(i-Tss*faii*3)/(0.5*Tss*faii))*pi/180;
    yaw=D*sin(2*pi*(i-Tss*faii*3)/(0.25*Tss*faii))*pi/180;
    roll=0;
}      

  double LL = 315; double ww = 140; H = 180;
  double  dx1 = LL / 2, dy1 = -H, dz1 = -ww / 2;
  x4 = -(cos(pitch) * cos(yaw) * dx1 - sin(pitch) * dy1 + cos(pitch) * sin(yaw) * dz1 - LL / 2);
  y4 =  -(dx1 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - dz1 * (cos(roll) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + cos(pitch) * cos(roll) * dy1);
  z4 = -(dz1 * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - dx1 * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(pitch) * sin(roll) * dy1 + ww / 2) + 38;

  double dx2 = LL / 2, dy2 = -H, dz2 = ww / 2;
  x3 = -(cos(pitch) * cos(yaw) * dx2 - sin(pitch) * dy2 + cos(pitch) * sin(yaw) * dz2 - LL / 2);
  y3 = -(dx2 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - dz2 * (cos(roll) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + cos(pitch) * cos(roll) * dy2);
  z3 = (dz2 * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - dx2 * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(pitch) * sin(roll) * dy2 - ww / 2) + 38;

  double dx3 = -LL / 2, dy3 = -H, dz3 = ww / 2;
  x2 = -(cos(pitch) * cos(yaw) * dx3 - sin(pitch) * dy3 + cos(pitch) * sin(yaw) * dz3 + LL / 2);
  y2 = -(dx3 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - dz3 * (cos(roll) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + cos(pitch) * cos(roll) * dy3);
  z2 = (dz3 * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - dx3 * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(pitch) * sin(roll) * dy3 - ww / 2) + 38;

  double  dx4 = -LL / 2, dy4 = -H, dz4 = -ww / 2;
  x1 = -(cos(pitch) * cos(yaw) * dx4 - sin(pitch) * dy4 + cos(pitch) * sin(yaw) * dz4 + LL / 2);
  y1 = -(dx4 * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - dz4 * (cos(roll) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + cos(pitch) * cos(roll) * dy4);
  z1 = -(dz4 * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - dx4 * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + cos(pitch) * sin(roll) * dy4 + ww / 2) + 38;
  int A=30;
  ik_3dof(l2, l3, x1 - A, x2 - A, x3 - A, x4 - A, y1, y2, y3, y4, z1, z2, z3, z4);
  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8], G[9], G[10], G[11]);
}

double Flag_trot = 0, Flag_walk = 0, Flag_stand = 0,Flag_balance=0,Flag_bend=0,Flag_Walk_bend=0,Flag_dance=0,Flag_jump=0;

void setup() {
  t = 0;
  Serial.begin(9600);
  mine.begin(9600);
  pinMode(13, OUTPUT);
  xs = 25; xf = 60;
  Ts_walk = 0.2; Ts = 0.1;

  Flag_stand = 0; Flag_trot = 0;

  JY901.StartIIC();
}


void stand_up_run()
{
  if (t < Ts_s) t += sspeed;
  stand_up(t);
  ik_3dof(l2, l3, x1, x2, x3, x4, y1, y2, y3, y4, z1, z2, z3, z4);
  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8], G[9], G[10], G[11]);
}
void jump_run()
{
  if (t < Ts_s+1.5) t += sspeed;
  else t=0;
  jump(t);
  ik_3dof(l2, l3, x1, x2, x3, x4, y1, y2, y3, y4, z1, z2, z3, z4);
  Move(G[0], G[1], G[2], G[3], G[4], G[5], G[6], G[7], G[8], G[9], G[10], G[11]);
}


void loop() {

  //bend_run();

  if (Flag_trot == 1)
  {
    Trot_run();
  }
  if (Flag_stand == 1)
  {
    stand_run();
  }
  if (Flag_walk == 1)
  {
    Walk_run();
  }
    if (Flag_balance == 1)
  {
    Balance_run();
  }
    if (Flag_bend == 1)
  {
    bend_run();
  }
    if (Flag_Walk_bend == 1)
  {
    Walk_bend_run();
  }
   if (Flag_dance == 1)
  {
    dance_run();
  }
   if (Flag_jump == 1)
  {
    jump_run();
  }
  
    char data;

     if(mine.available())
     {
       data = mine.read();
       switch(data)
       {
         case '0': r1=0,r2=0,r3=0,r4=0;break;
         case '1': r1=1,r2=1,r3=1,r4=1;break;
         case '2': r1=0,r2=1,r3=0,r4=1;break;
         case '3': r1=1,r2=0,r3=1,r4=0;break;
         case '4': r1=-1,r2=-1,r3=-1,r4=-1;break;
         case '5':up();break;
         case '6':setServo(3,60);break;
         case '7':setServo(3,150);break;
         case '8':Flag_jump=1,Flag_trot = 0, Flag_walk = 0, Flag_stand = 0,Flag_balance=0,Flag_bend=0,Flag_Walk_bend=0,Flag_dance=0;break;
         case '9':Flag_jump=0,Flag_trot = 0, Flag_walk = 0, Flag_stand = 0,Flag_balance=0,Flag_bend=0,Flag_Walk_bend=0,Flag_dance=1;break;
         case 'a':Flag_jump=0,Flag_Walk_bend=1,Flag_trot = 0, Flag_walk = 0, Flag_stand = 0,Flag_balance=0,Flag_bend=0,Flag_dance=0;break;
         case 'b':Flag_jump=0,Flag_trot = 0, Flag_walk = 0, Flag_stand = 0,Flag_balance=0,Flag_bend=1,Flag_Walk_bend=0,Flag_dance=0;break;
         case 'c':Flag_jump=0,Flag_trot = 1, Flag_walk = 0, Flag_stand = 0,Flag_balance=0,Flag_bend=0,Flag_Walk_bend=0,Flag_dance=0;break;
         case 'd':Flag_jump=0,Flag_trot = 0, Flag_walk = 1, Flag_stand = 0,Flag_balance=0,Flag_bend=0,Flag_Walk_bend=0,Flag_dance=0;break;
         case 'e':Flag_jump=0,Flag_trot = 0, Flag_walk = 0, Flag_stand = 1,Flag_balance=0,Flag_bend=0,Flag_Walk_bend=0,Flag_dance=0;break;
         case 'f':Flag_jump=0,Flag_trot = 0, Flag_walk = 0, Flag_stand = 0,Flag_balance=1,Flag_bend=0,Flag_Walk_bend=0,Flag_dance=0;break;
       }
     }

}

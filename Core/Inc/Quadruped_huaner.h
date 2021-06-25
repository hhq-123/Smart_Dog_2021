#ifndef QUADRYPED_HUANER_H_
#define QUADRYPED_HUANER_H_

#include "stm32f401xc.h"

#define Move_t 1
#define LF_wai_Init  1560
#define LF_ham_Init  1383
#define LF_shank_Init  1589
#define RF_wai_Init  1434
#define RF_ham_Init  1510
#define RF_shank_Init  1470
#define RB_wai_Init  1452
#define RB_ham_Init  1400
#define RB_shank_Init  1470
#define LB_wai_Init  1460
#define LB_ham_Init  1394
#define LB_shank_Init  1569


#define H 150
#define d 60
#define l2 130
#define l3 111
#define hh 50
#define PI 3.1415926535
#define RtoA  180 / PI

#define Shank_a 40
#define Shank_b 70
#define Shank_c 39
#define Shank_d 65
#define Shank_e 25

void Servo_Init(void);
void Move(double ham[],double shank[], double wai[],  uint16_t Time);
void Move_zero(uint16_t Time);
void ik_3dof(double x[], double y[], double z[]);
void ik_Move(double x[], double y[], double z[]);
void gait_Init(void);
void LSC_communication(void);
#endif

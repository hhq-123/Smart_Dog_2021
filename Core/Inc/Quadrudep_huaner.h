#ifndef QUADRYDEP_HUANER_H_
#define QUADRYDEP_HUANER_H_

#define Move_t 1
#define LF_wai_Init  1560
#define LF_ham_Init  1396
#define LF_shank_Init  1500//1589
#define RF_wai_Init  1442
#define RF_ham_Init  1510
#define RF_shank_Init  1500//1340
#define RB_wai_Init  1476
#define RB_ham_Init  1400
#define RB_shank_Init  1500//1578
#define LB_wai_Init  1448
#define LB_ham_Init  1480
#define LB_shank_Init  1500//1569

#define H 180
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
void Move(double ham[],double shank[], double wai[]);
void ik_3dof(double x[], double y[], double z[]);
void ik_Move(double x[], double y[], double z[]);

#endif

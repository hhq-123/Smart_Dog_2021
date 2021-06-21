#ifndef TEST_WORKSPACE_H_
#define TEST_WORKSPACE_H_

typedef struct {
  double kp, ki, kd;
  double error, output,last_error, integral;
} PId_struct_typedef;

void gait_trot(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3);
void gait_walk(double t, double xs, double xf, double h, double r1, double r2, double r3, double r4);
void gait_crab(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3);
void gait_round(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3);
void gait_step(int t, double h, double zero_step);
void balance(double PItch2, double roll2, double yaw);

void Trot_state(double sp, double xs, double xf, double h, double r1, double r2, double r3, double r4);
void Walk_state(double sp, double xs, double xf, double h, double r1, double r2, double r3, double r4);
void Crab_state(double  sp, double xs, double xf, double h, double r1, double r2, double r3, double r4);
void Round_state(double  sp, double xs, double xf, double h, double r1, double r2, double r3, double r4);
void Step_state(double sp, double h, double zero_step);
void Balance_state(double sp);

void Trot_run(void);
void Walk_run(void);
void Stand_run(void);
void Dance_run(void);
void Bend_run(void);
void Balance_run(void);
void Walk_Bend_run(void);
void Crab_L_run(void);
void Crab_R_run(void);
void Round_L_run(void);
void Round_R_run(void);
void Step_run(void);
void Stand_Up_run(void);
void Power_down(void);

#endif

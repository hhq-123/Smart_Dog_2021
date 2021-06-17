#ifndef TEST_WORKSPACE_H_
#define TEST_WORKSPACE_H_

void gait_trot(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3);
void gait_crab(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3);
void gait_round(int t, double xs, double xf, double h, double r1, double r4, double r2, double r3);

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
void Stand_Up_run(void);
void Power_down(void);

#endif

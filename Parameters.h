#ifndef __PARAMETERS__
#define __PARAMETERS__
#include "FC_PI.h"
#include "FC_PID.h"
#include "RC_Channel.h"
extern FC_PI pi_stabilize_roll; // = {STABILIZE_ROLL_P, STABILIZE_ROLL_I, STABILIZE_ROLL_IMAX, 0};
extern FC_PI pi_stabilize_pitch; // = {STABILIZE_PITCH_P, STABILIZE_PITCH_I, STABILIZE_PITCH_IMAX, 0};
extern FC_PI pi_stabilize_yaw; // = {STABILIZE_YAW_P, STABILIZE_YAW_I, STABILIZE_YAW_IMAX, 0};
//float acro_p = ACRO_P;
extern FC_PID pid_rate_roll; // = {RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, RATE_ROLL_IMAX};
extern FC_PID pid_rate_pitch; // = {RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX};
extern FC_PID pid_rate_yaw; // = {RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX};
extern FC_PI pi_stabilize_up;
extern FC_PID ac_pid_height;
extern FC_PID ac_pid_point_x;
extern FC_PID ac_pid_point_y;
    ////extern float acro_p;
//这几个我暂时用不到		
// extern APM_PI pi_alt_hold;
// extern AC_PID pid_throttle;
// extern int throttle_cruise;

extern RC_Channel rc_1, rc_2, rc_3, rc_4;

void _Parameters_init(void);


#endif

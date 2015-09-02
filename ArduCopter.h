#ifndef __ARDU_COPTER__
#define __ARDU_COPTER__
#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_AHRS_DCM.h"
#include "AP_MotorsQuard.h"
extern Vector3f current_omega;
extern AP_AHRS_DCM ap_ahrs_dcm;
extern int begin_fly;
extern long height_current;
extern long height_target;
extern long target_roll_degree, target_pitch_degree, target_yaw_degree;
extern AP_MotorsQuard ap_motors_quard;
extern AP_AHRS_DCM ap_ahrs_dcm;
extern long current_point_x,current_point_y;
extern int alt_hold;
extern float G_Dt;
extern long 	target_point_x ,target_point_y;
extern int hover_hold;
extern int height_base;
//
void arducopter_init(void);
void arducopter_fast_loop(void);

void do_task(void);
//
void read_AHRS(void);
void read_inertia(void);
void update_yaw_mode(void);
void update_roll_pitch_mode(void);
void update_rate_contoller_targets(void);
void update_throttle_servo_out(void);


void get_stabilize_roll(long target_angle);
void get_stabilize_pitch(long target_angle);
void get_stabilize_yaw(long target_angle);
void run_rate_controllers(void);
void set_servos_4(void);

int get_rate_roll(long target_rate);
int get_rate_pitch(long target_rate);
int get_rate_yaw(long target_rate);

void move_forward(void);
void move_backward(void);
void middle_pitch(void);
void move_left(void);
void middle_roll(void);
void move_right(void);

void do_basic_low_height_task(void);

void update_point_control(void);



enum TaskType
{
	basic_low_height,
	basic_high_height,
	super_low_height,
	super_high_height,
	keep_height
};

#endif

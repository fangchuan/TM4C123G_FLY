#include "FC_PI.h"
#include "FC_PID.h"
#include "RC_Channel.h"
#include "confige.h"
#include "Parameters.h"
FC_PI pi_stabilize_roll; // = {STABILIZE_ROLL_P, STABILIZE_ROLL_I, STABILIZE_ROLL_IMAX, 0};
FC_PI pi_stabilize_pitch; // = {STABILIZE_PITCH_P, STABILIZE_PITCH_I, STABILIZE_PITCH_IMAX, 0};
FC_PI pi_stabilize_yaw; // = {STABILIZE_YAW_P, STABILIZE_YAW_I, STABILIZE_YAW_IMAX, 0};
//float acro_p = ACRO_P;
FC_PID pid_rate_roll; // = {RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, RATE_ROLL_IMAX};
FC_PID pid_rate_pitch; // = {RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX};
FC_PID pid_rate_yaw; // = {RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX};
FC_PI pi_stabilize_up;
FC_PID ac_pid_height;
FC_PID ac_pid_point_x;
FC_PID ac_pid_point_y;
RC_Channel rc_1;
RC_Channel rc_2;
RC_Channel rc_3;
RC_Channel rc_4;

void _Parameters_init(void)
{
	_FC_PI_init(&pi_stabilize_roll, STABILIZE_ROLL_P, STABILIZE_ROLL_I, STABILIZE_ROLL_IMAX);
	_FC_PI_init(&pi_stabilize_pitch, STABILIZE_PITCH_P, STABILIZE_PITCH_I, STABILIZE_PITCH_IMAX);
	_FC_PI_init(&pi_stabilize_yaw, STABILIZE_YAW_P, STABILIZE_YAW_I, STABILIZE_YAW_IMAX);
	
	fc_pid_init(&pid_rate_roll, RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, RATE_ROLL_IMAX);
	fc_pid_init(&pid_rate_pitch, RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX);
	fc_pid_init(&pid_rate_yaw, RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX);
	
		//init height pid

	fc_pid_init(&ac_pid_height, HEIGHT_P, HEIGHT_I, HEIGHT_D, HEIGHT_I_MAX);
	//init  poisition pid
	fc_pid_init(&ac_pid_point_x,POINT_X_P,POINT_X_I,POINT_X_D,POINT_X_I_MAX);
	fc_pid_init(&ac_pid_point_y,POINT_Y_P,POINT_Y_I,POINT_Y_D,POINT_Y_I_MAX);
	
	rc_channel_init(&rc_1);//roll 通道初始化
	rc_channel_set_angle(&rc_1, MAX_INPUT_ROLL_ANGLE);//set the biggest 
	rc_1.type = RC_CHANNEL_TYPE_ANGLE_RAW;

	rc_channel_init(&rc_2);//pitch 通道初始化
	rc_channel_set_angle(&rc_2, MAX_INPUT_PITCH_ANGLE);
	rc_2.type = RC_CHANNEL_TYPE_ANGLE_RAW;

	rc_channel_init(&rc_3);
	rc_channel_set_range(&rc_3, MINIMUM_THROTTLE, MAXIMUM_THROTTLE);

	rc_channel_init(&rc_4);//yaw 通道初始化
	rc_channel_set_angle(&rc_4, MAX_INPUT_YAW_ANGLE);
	rc_4.type = RC_CHANNEL_TYPE_ANGLE_RAW;
}
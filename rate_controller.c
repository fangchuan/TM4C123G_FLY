#include "TM4C.h"
#include "AP_Math.h"
#include "FC_PI.h"
#include "FC_PID.h"
#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_AHRS_DCM.h"
#include "RC_Channel.h"
#include "AP_MotorsQuard.h"
#include "Parameters.h"
#include "math.h"
//extern unsigned char r[6],p[6],y[6];
extern AP_AHRS_DCM ap_ahrs_dcm;
extern float G_Dt;
extern long roll_rate_target_ef, pitch_rate_target_ef, yaw_rate_target_ef;
extern long roll_rate_target_bf, pitch_rate_target_bf, yaw_rate_target_bf;
extern Vector3f current_omega;
extern AP_MotorsQuard ap_motors_quard;
//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) 
// long wrap_180(long error)
// {
// 	if(error>18000) error-=36000;
// 	if(error<-18000) error+=36000;
// 	return error;
// }
void get_stabilize_roll(long target_angle)
{
	long target_rate;
	long i_stab;

	// angle error
	target_angle = wrap_180(target_angle - ap_ahrs_dcm.roll_sensor);//期望角度-当前角度

	// limit the error we're feeding to the PID
	target_angle = constrain(target_angle, -4500, 4500);//误差在+-45之间

	// convert to desired Rate:
	target_rate = _FC_PI_get_p(&pi_stabilize_roll, target_angle);

//	i_stab;
	if(fabs(ap_ahrs_dcm.roll_sensor) < 500) {
		target_angle = constrain(target_angle, -500, 500);
		i_stab = _FC_PI_get_i(&pi_stabilize_roll, target_angle, G_Dt);
	}else{
		i_stab = (long)pi_stabilize_roll.integrator;
	}

	// set targets for rate controller
	roll_rate_target_ef = target_rate + i_stab;

}

void get_stabilize_pitch(long target_angle)
{
	long target_rate;
	long i_stab;

	// angle error
	target_angle = wrap_180(target_angle - ap_ahrs_dcm.pitch_sensor);

	// limit the error we're feeding to the PID
	target_angle = constrain(target_angle, -4500, 4500);

	// convert to desired Rate:
	target_rate = _FC_PI_get_p(&pi_stabilize_pitch, target_angle);

//	i_stab;
	if(fabs(ap_ahrs_dcm.pitch_sensor) < 500) {
		target_angle = constrain(target_angle, -500, 500);
		i_stab = _FC_PI_get_i(&pi_stabilize_pitch, target_angle, G_Dt);
	}else{
		i_stab = (long)pi_stabilize_pitch.integrator;
	}

	// set targets for rate controller
	pitch_rate_target_ef = target_rate + i_stab;

}

void get_stabilize_yaw(long target_angle)
{
	long target_rate,i_term;
	long angle_error;
	long output = 0;

	// angle error
	angle_error = wrap_180(target_angle - ap_ahrs_dcm.yaw_sensor);//应该是转换成180对应的弧度制

	// limit the error we're feeding to the PID
	angle_error = constrain(angle_error, -4500, 4500);//ensure the error are 可信的

	// convert angle error to desired Rate:
	target_rate = _FC_PI_get_p(&pi_stabilize_yaw, angle_error);
	i_term = _FC_PI_get_i(&pi_stabilize_yaw, angle_error, G_Dt);

	// set targets for rate controller
	yaw_rate_target_ef = target_rate + i_term;

}

int get_rate_roll(long target_rate)
{
	long p,i,d;                  // used to capture pid values for logging
	long current_rate;           // this iteration's rate
	long rate_error;             // simply target_rate - current_rate
	long output;                 // output from pid controller

	// get current rate
	current_rate    = (long)(current_omega.x * DEGX100);
	// call pid controller
	rate_error  = target_rate - current_rate;
	p = fc_pid_get_p(&pid_rate_roll, rate_error);

	// freeze I term if we've breached roll-pitch limits
	if(ap_motors_quard_reached_limit(&ap_motors_quard, AP_MOTOR_ROLLPITCH_LIMIT))
	{
		i = (long)pid_rate_roll.integrator;
	}else{
		i = (long)fc_pid_get_i(&pid_rate_roll, rate_error, G_Dt);
	}

	d = fc_pid_get_d(&pid_rate_roll, rate_error, G_Dt);
	output = p + i + d;

	// constrain output
	output = constrain(output, -5000, 5000);

	// output control
	return output;
}

int get_rate_pitch(long target_rate)
{
	long p,i,d;                  // used to capture pid values for logging
	long current_rate;           // this iteration's rate
	long rate_error;             // simply target_rate - current_rate
	long output;                 // output from pid controller

	// get current rate
	current_rate    = (long)(current_omega.y * DEGX100);

	// call pid controller
	rate_error  = target_rate - current_rate;
	p = fc_pid_get_p(&pid_rate_pitch, rate_error);

	// freeze I term if we've breached roll-pitch limits
	if(ap_motors_quard_reached_limit(&ap_motors_quard, AP_MOTOR_ROLLPITCH_LIMIT))
	{
		i = (long)pid_rate_pitch.integrator;
	}else{
		i = (long)fc_pid_get_i(&pid_rate_pitch, rate_error, G_Dt);
		
	}

	d = fc_pid_get_d(&pid_rate_pitch, rate_error, G_Dt);
	//d=constrain(d,-5000,5000);
	output = p + i + d;

	// constrain output
	output = constrain(output, -5000, 5000);

	// output control
	return output;
}

int get_rate_yaw(long target_rate)
{
	long p,i,d;                  // used to capture pid values for logging
	long current_rate;           // this iteration's rate
	long rate_error;             // simply target_rate - current_rate
	long output;                 // output from pid controller
	//int yaw_limit;

	// get current rate
	current_rate    = (long)(current_omega.z * DEGX100);

	// call pid controller
	rate_error  = target_rate - current_rate;
	p = fc_pid_get_p(&pid_rate_yaw, rate_error);

	// freeze I term if we've breached roll-pitch limits
	if(ap_motors_quard_reached_limit(&ap_motors_quard, AP_MOTOR_YAW_LIMIT))
	{
		i = (long)pid_rate_yaw.integrator;
	}else{
		i = fc_pid_get_i(&pid_rate_yaw, rate_error, G_Dt);
	}

	d = fc_pid_get_d(&pid_rate_yaw, rate_error, G_Dt);
	output = p + i + d;
  
	// constrain output
	output = constrain(output, -4500, 4500);

	//yaw_limit = 2200 + abs(g.rc_4.control_in);

	// smoother Yaw control:
	//return constrain(output, -yaw_limit, yaw_limit);

	return output;
}

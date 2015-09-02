
#include "FC_PID.h"
#include <math.h>
#include "AP_Math.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
void fc_pid_init(FC_PID * ap_pid, float initial_p, float initial_i, float initial_d, float initial_imax)
{
	ap_pid->kp = initial_p;//????
	ap_pid->ki = initial_i;//????
	ap_pid->kd = initial_d;//????
	ap_pid->imax = fabsf(initial_imax);//????????

	ap_pid->integrator = 0;//???
	ap_pid->is_last_derivative_valid = 0;//???????,?????
}

long fc_pid_get_pid(FC_PID * ap_pid, long error, float dt)
{
	 return fc_pid_get_p(ap_pid, error) + fc_pid_get_i(ap_pid, error, dt) + fc_pid_get_d(ap_pid, error, dt); 
}

long fc_pid_get_pi(FC_PID * ap_pid, long error, float dt)
{
	return fc_pid_get_p(ap_pid, error) + fc_pid_get_i(ap_pid, error, dt);
}

long fc_pid_get_p(FC_PID * ap_pid, long error)
{
	 return (long)(error * ap_pid->kp);
}

long fc_pid_get_i(FC_PID * ap_pid, long error, float dt)
{
	if((ap_pid->ki != 0) && (dt != 0)) {
		ap_pid->integrator += ((float)error * ap_pid->ki) * dt;
		if (ap_pid->integrator < -ap_pid->imax) {
			ap_pid->integrator = -ap_pid->imax;
		} else if (ap_pid->integrator > ap_pid->imax) {
			ap_pid->integrator = ap_pid->imax;
		}
		return (long)ap_pid->integrator;
	}
	return 0;
}
//#include "debug.h"
long fc_pid_get_d(FC_PID * ap_pid, long error, float dt)
{
	if ((ap_pid->kd != 0) && (dt != 0)) {
		float derivative;
		if (!ap_pid->is_last_derivative_valid) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			ap_pid->last_derivative = 0;
			ap_pid->is_last_derivative_valid = 1;
		} else {
			// calculate instantaneous derivative
			derivative = (error - ap_pid->last_input) / dt;
		}
		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		derivative = ap_pid->last_derivative +
			(dt / ( FC_PID_FILTER + dt)) * (derivative - ap_pid->last_derivative);
      //debug_printf("derivative:%f\n",derivative);;
		// update state
		ap_pid->last_input             = error;
		ap_pid->last_derivative    = derivative;
      
		// add in derivative component
		return (long)(ap_pid->kd * derivative);
	}
	return 0;
}

long fc_pid_get_d2(FC_PID * ap_pid, long error, float dt)
{
	int flag1=0;
	
	if ((ap_pid->kd != 0) && (dt != 0)) {
		float derivative;
		if (!ap_pid->is_last_derivative_valid) {
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			ap_pid->last_derivative = 0;
			ap_pid->is_last_derivative_valid = 1;
		} else {
			// calculate instantaneous derivative
			if(fabs(error) - fabs(ap_pid->last_input) >= 0)
			{				flag1=1;
				      derivative = -(error - ap_pid->last_input) / dt;
			}
			else 
				derivative = (error - ap_pid->last_input) / dt;
		}
		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		derivative = ap_pid->last_derivative +
			(dt / ( FC_PID_FILTER + dt)) * (derivative - ap_pid->last_derivative);
      //debug_printf("derivative:%f\n",derivative);;
		// update state
		ap_pid->last_input             = error;
		ap_pid->last_derivative    = derivative;
      
		// add in derivative component
// 		if(flag1==1)
// 		return (long)(-ap_pid->kd * derivative);
// 		else
		return (long)(ap_pid->kd * derivative);
	}
	return 0;
}
long fc_pid_get_leaky_i(FC_PID * ap_pid, long error, float dt, float leak_rate)
{
	if((ap_pid->ki != 0) && (dt != 0)){
		ap_pid->integrator -= (float)ap_pid->integrator * leak_rate;
		ap_pid->integrator += ((float)error * ap_pid->ki) * dt;
		if (ap_pid->integrator < -ap_pid->imax) {
			ap_pid->integrator = -ap_pid->imax;
		} else if (ap_pid->integrator > ap_pid->imax) {
			ap_pid->integrator = ap_pid->imax;
		}

		return (long)(ap_pid->integrator);
	}
	return 0;
}


/// Reset the PID integrator
///
void fc_pid_reset_I(FC_PID * ap_pid)
{
	ap_pid->integrator = 0;
	// mark derivative as invalid
	ap_pid->is_last_derivative_valid = 0;
}

long calc_pid_position(FC_PID*ac_pid, long error,float dt)  //Î»ÖÃ¿ØÖÆPID
{
		long p_o,i_o,d_o;
    static long last_error;
			p_o = error*ac_pid->kp;							
			i_o += error*ac_pid->ki*dt;
			if(i_o>=ac_pid->imax)	
         i_o = ac_pid->imax;
      else
          if(i_o <= -ac_pid->imax)
             i_o = -ac_pid->imax;						
			d_o = (error - last_error)*ac_pid->kd/dt;
			
			last_error=error;
					
			return ( p_o + i_o + d_o);
}
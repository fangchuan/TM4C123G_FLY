#ifndef __FC_PID__
#define __FC_PID__

typedef struct _FC_PID
{
	float kp, ki, kd, imax;
	
	float integrator;                      ///< integrator value
	long last_input;                      ///< last input for derivative上一次误差，用于微分项
	float last_derivative;                ///< last derivative for low-pass filter//error(k-2)
	int is_last_derivative_valid;
} FC_PID;

#define FC_PID_FILTER 7.9577e-3F  // Set to  "1 / ( 2 * PI * f_cut )"

void fc_pid_init(FC_PID * ap_pid, float initial_p, float initial_i, float initial_d, float initial_imax);
long fc_pid_get_pid(FC_PID * ap_pid, long error, float dt);
long fc_pid_get_pi(FC_PID * ap_pid, long error, float dt);
long fc_pid_get_p(FC_PID * ap_pid, long error);
long fc_pid_get_i(FC_PID * ap_pid, long error, float dt);
long fc_pid_get_d(FC_PID * ap_pid, long error, float dt);
long fc_pid_get_d2(FC_PID * ap_pid, long error, float dt);
long fc_pid_get_leaky_i(FC_PID * ap_pid, long error, float dt, float leak_rate);
void fc_pid_reset_I(FC_PID * ap_pid);
long calc_pid_position(FC_PID*ac_pid, long error,float dt);
#endif

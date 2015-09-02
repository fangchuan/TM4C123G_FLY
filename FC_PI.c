#include "FC_PI.h"
//P\I参数初始化
void _FC_PI_init(FC_PI *apm_pi, float kp, float ki, float imax)
{
	apm_pi->kp = kp;
	apm_pi->ki = ki;
	apm_pi->imax = imax;
	apm_pi->integrator = 0;
}

long _FC_PI_get_p(FC_PI *apm_pi, long error)
{
	return (long)(error * apm_pi->kp);
}

long _FC_PI_get_i(FC_PI *apm_pi, long error, float dt)
{
	if(dt != 0) {
		apm_pi->integrator += ((float)error * apm_pi->ki) * dt;

		if (apm_pi->integrator < -apm_pi->imax) {
			apm_pi->integrator = (float)-apm_pi->imax;
		} else if (apm_pi->integrator > apm_pi->imax) {
			apm_pi->integrator = (float)apm_pi->imax;
		}
	}
	return (long)apm_pi->integrator;
}

long _FC_PI_get_pi(FC_PI *apm_pi, long error, float dt)
{
	return _FC_PI_get_p(apm_pi, error) + _FC_PI_get_i(apm_pi, error, dt);
}
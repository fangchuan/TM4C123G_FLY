//FC_PI.H里面的PID结构体只定义了比例项、积分项、积分项上限、积分和
#ifndef __FC_PI__
#define __FC_PI__

typedef struct _APM_PI
{
	float kp;//比例项系数
	float ki;//积分项系数
	float imax;//积分和上限
	float integrator;//积分和，用于误差累积
} FC_PI;

void _FC_PI_init(FC_PI *apm_pi, float kp, float ki, float imax);
long _FC_PI_get_p(FC_PI *apm_pi, long error);
long _FC_PI_get_i(FC_PI *apm_pi, long error, float dt);
long _FC_PI_get_pi(FC_PI *apm_pi, long error, float dt);


#endif

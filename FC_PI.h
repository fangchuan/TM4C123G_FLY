//FC_PI.H�����PID�ṹ��ֻ�����˱������������������ޡ����ֺ�
#ifndef __FC_PI__
#define __FC_PI__

typedef struct _APM_PI
{
	float kp;//������ϵ��
	float ki;//������ϵ��
	float imax;//���ֺ�����
	float integrator;//���ֺͣ���������ۻ�
} FC_PI;

void _FC_PI_init(FC_PI *apm_pi, float kp, float ki, float imax);
long _FC_PI_get_p(FC_PI *apm_pi, long error);
long _FC_PI_get_i(FC_PI *apm_pi, long error, float dt);
long _FC_PI_get_pi(FC_PI *apm_pi, long error, float dt);


#endif

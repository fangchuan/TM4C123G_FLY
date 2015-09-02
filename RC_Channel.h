#ifndef __RC_CHANNEL__
#define __RC_CHANNEL__

#define RC_CHANNEL_TYPE_ANGLE       0
#define RC_CHANNEL_TYPE_RANGE       1
#define RC_CHANNEL_TYPE_ANGLE_RAW   2

#define RADIO_MIN	1100  //这部分油门的偏移不要改，这只是将每个通道等分成1000分。PID将每个通道的控制量综合计算后送至PWM最终输出
#define RADIO_TRIM	1500//要改只能改具体实现部分即PWM设置
#define RADIO_MAX	1900

typedef struct _RC_Channel
{
	int servo_out;

	// PWM is without the offset from radio_min
	int pwm_out;
	int radio_out;

	int radio_min;
	int radio_trim;
	int radio_max;

	int reverse;

	int high;
	int high_out;
	int low;
	int low_out;

	int type;
} RC_Channel;

void rc_channel_init(RC_Channel *rc_channel);
void rc_channel_calc_pwm(RC_Channel *rc_channel);
void rc_channel_set_range(RC_Channel *rc_channel, int low, int high);
void rc_channel_set_angle(RC_Channel *rc_channel, int angle);


#endif


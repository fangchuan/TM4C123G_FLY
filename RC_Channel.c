#include "RC_Channel.h"
#include "AP_Math.h"
#include "ArduCopter.h"
#include "TM4C.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
extern unsigned char r[6];
int rc_channel_angle_to_pwm(RC_Channel *rc_channel);
int	rc_channel_range_to_pwm(RC_Channel *rc_channel);

void rc_channel_init(RC_Channel *rc_channel)
{
	rc_channel->servo_out = 0;
	rc_channel->reverse = 1;
	rc_channel->high = 1;

	rc_channel->radio_min = RADIO_MIN;
	rc_channel->radio_trim = RADIO_TRIM;
	rc_channel->radio_max = RADIO_MAX;
}

void rc_channel_set_range(RC_Channel *rc_channel, int low, int high)
{
	rc_channel->type  = RC_CHANNEL_TYPE_RANGE;
	rc_channel->high = high;
	rc_channel->low = low;
	rc_channel->high_out = high;
	rc_channel->low_out = low;
}

void rc_channel_set_angle(RC_Channel *rc_channel, int angle)
{
	rc_channel->type = RC_CHANNEL_TYPE_ANGLE;
	rc_channel->high = angle;
}

void rc_channel_calc_pwm(RC_Channel *rc_channel)//PWM的算法
{
	if(rc_channel->type == RC_CHANNEL_TYPE_RANGE) {
		rc_channel->pwm_out = rc_channel_range_to_pwm(rc_channel);
		rc_channel->radio_out  = //(rc_channel->reverse >= 0) ? 
			(rc_channel->radio_min + rc_channel->pwm_out);// : (rc_channel->radio_max - rc_channel->pwm_out);

	}else if(rc_channel->type == RC_CHANNEL_TYPE_ANGLE_RAW) //将角速度控制量*0.1
	{
		rc_channel->pwm_out  = (int)(rc_channel->servo_out * 0.1);
		rc_channel->radio_out = (rc_channel->pwm_out * rc_channel->reverse) + rc_channel->radio_trim;

	}
//   UARTprintf("rc_channel->pwm_out:");long_to_string(r,rc_channel->pwm_out);UARTCharPut(UART5_BASE,r[0]);UARTCharPut(UART5_BASE,r[1]);
//   UARTCharPut(UART5_BASE,r[2]);UARTCharPut(UART5_BASE,r[3]);UARTCharPut(UART5_BASE,r[4]);UARTCharPut(UART5_BASE,r[5]);UARTprintf("\r\n");
	rc_channel->radio_out = constrain(rc_channel->radio_out, rc_channel->radio_min, rc_channel->radio_max);
}

int	rc_channel_range_to_pwm(RC_Channel *rc_channel)
{
	return ((long)(rc_channel->servo_out - rc_channel->low_out) * 
		(long)(rc_channel->radio_max - rc_channel->radio_min)) / (long)(rc_channel->high_out - rc_channel->low_out);
}

int rc_channel_angle_to_pwm(RC_Channel *rc_channel)
{
	if((rc_channel->servo_out * rc_channel->reverse) > 0)
		return rc_channel->reverse * ((long)rc_channel->servo_out * (long)(rc_channel->radio_max - rc_channel->radio_trim)) / (long)rc_channel->high;
	else
		return rc_channel->reverse * ((long)rc_channel->servo_out * (long)(rc_channel->radio_trim - rc_channel->radio_min)) / (long)rc_channel->high;
}
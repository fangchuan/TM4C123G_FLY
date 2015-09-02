#include "AP_Math.h"
#include "RC_Channel.h"
#include "TM4C.h"
#include "AP_MotorsQuard.h"
#include <math.h>
#include "PWM.h"
#include "ArduCopter.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
extern unsigned char r[6];

void ap_motors_quard_add_motor_raw(AP_MotorsQuard *ap_motors_quard, int motor_num, float roll_fac, float pitch_fac, float yaw_fac);
void ap_motors_quard_add_motor(AP_MotorsQuard *ap_motors_quard, int motor_num, float angle_degrees, int direction);


void ap_motors_quard_init(AP_MotorsQuard *ap_motors_quard, RC_Channel *rc_roll, RC_Channel *rc_pitch, 
	RC_Channel *rc_throttle, RC_Channel *rc_yaw)
{//根据得到的三个通道角速度控制量初始化电机
	int i;
 
	ap_motors_quard->reached_limit = AP_MOTOR_NO_LIMITS_REACHED;
	ap_motors_quard->max_throttle = AP_MOTORS_DEFAULT_MAX_THROTTLE;//电机的最大油门
	ap_motors_quard->min_throttle = AP_MOTORS_DEFAULT_MIN_THROTTLE;//电机的最小油门

	ap_motors_quard->rc_roll = rc_roll;//
	ap_motors_quard->rc_pitch = rc_pitch;//
	ap_motors_quard->rc_throttle = rc_throttle;//
	ap_motors_quard->rc_yaw = rc_yaw;

	for(i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
	{
		ap_motors_quard->motor_out[i] = 0;
	}
  //X model
	ap_motors_quard_add_motor(ap_motors_quard, AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_MOTOR_CCW);//1、2电机正转
	ap_motors_quard_add_motor(ap_motors_quard, AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_MOTOR_CCW);
	ap_motors_quard_add_motor(ap_motors_quard, AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_MOTOR_CW);//34dianji fanzhuan
	ap_motors_quard_add_motor(ap_motors_quard, AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_MOTOR_CW);
}

void ap_motors_quard_add_motor_raw(AP_MotorsQuard *ap_motors_quard, int motor_num, float roll_fac, float pitch_fac, float yaw_fac)
{
	ap_motors_quard->roll_factor[motor_num] = roll_fac;//use 电机与X轴夹角来判断电机的PWM应该是增还是剪
	ap_motors_quard->pitch_factor[motor_num] = pitch_fac;
	ap_motors_quard->yaw_factor[motor_num] = yaw_fac;
}

void ap_motors_quard_add_motor(AP_MotorsQuard *ap_motors_quard, int motor_num, float angle_degrees, int direction)
{
	ap_motors_quard_add_motor_raw(ap_motors_quard,motor_num,cosf(radians(angle_degrees + 90)),               // roll factor
		-cosf(radians(angle_degrees)),                    // pitch factor
		(float)direction                                // yaw factor
	);
}

unsigned char ap_motors_quard_reached_limit(AP_MotorsQuard *ap_motors_quard, unsigned char which_limit) {
	return ap_motors_quard->reached_limit & which_limit;
}

void ap_motors_quard_output_armed(AP_MotorsQuard *ap_motors_quard)
{
	int i;
	int out_min = ap_motors_quard->rc_throttle->radio_min;//
	int out_max = ap_motors_quard->rc_throttle->radio_max;
	int rc_yaw_constrained_pwm;
	int rc_yaw_excess;
	int upper_margin, lower_margin;
	int motor_adjustment = 0;
	int yaw_to_execute = 0;

	// initialize reached_limit flag
	ap_motors_quard->reached_limit = AP_MOTOR_NO_LIMITS_REACHED;

	// Throttle is 0 to 1000 only
	ap_motors_quard->rc_throttle->servo_out = constrain(ap_motors_quard->rc_throttle->servo_out, 0, ap_motors_quard->max_throttle);

	// capture desired roll, pitch, yaw and throttle from receiver
	// use servo out to calculate pwm_out and radio_out*******************很有道理但是没弄明白是怎么算的
	rc_channel_calc_pwm(ap_motors_quard->rc_roll);//
	rc_channel_calc_pwm(ap_motors_quard->rc_pitch);
	rc_channel_calc_pwm(ap_motors_quard->rc_throttle);
	rc_channel_calc_pwm(ap_motors_quard->rc_yaw);
  
	// if we are not sending a throttle output, we cut the motors 没有油门要求
	if(ap_motors_quard->rc_throttle->servo_out == 0) {
		for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
			ap_motors_quard->motor_out[i] = ap_motors_quard->rc_throttle->radio_min;
		}
		// if we have any roll, pitch or yaw input then it's breaching the limit
		if( ap_motors_quard->rc_roll->pwm_out != 0 || ap_motors_quard->rc_pitch->pwm_out != 0 ) {
			ap_motors_quard->reached_limit |= AP_MOTOR_ROLLPITCH_LIMIT;
		}
		if( ap_motors_quard->rc_yaw->pwm_out != 0 ) {
			ap_motors_quard->reached_limit |= AP_MOTOR_YAW_LIMIT;
		}
	} else {    // non-zero throttle
     //lowest throttle+default PWM 
		out_min = ap_motors_quard->rc_throttle->radio_min + ap_motors_quard->min_throttle;

		// initialise rc_yaw_contrained_pwm that we will certainly output and rc_yaw_excess that we will do on best-efforts basis.
		// Note: these calculations and many others below depend upon _yaw_factors always being 0, -1 or 1.
		if( ap_motors_quard->rc_yaw->pwm_out < -AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM ) {
			rc_yaw_constrained_pwm = -AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
			rc_yaw_excess = ap_motors_quard->rc_yaw->pwm_out + AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
		}else if( ap_motors_quard->rc_yaw->pwm_out > AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM ) {
			rc_yaw_constrained_pwm = AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
			rc_yaw_excess = ap_motors_quard->rc_yaw->pwm_out - AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM;
		}else{
			rc_yaw_constrained_pwm = ap_motors_quard->rc_yaw->pwm_out;
			rc_yaw_excess = 0;
		}

		// initialise upper and lower margins
		upper_margin = lower_margin = out_max - out_min;
//几个_factor[]被改写了，我也不知道在哪被改
		// add roll, pitch, throttle and constrained yaw for each motor
		for( i=0; i< AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
				ap_motors_quard->motor_out[i] = (ap_motors_quard->rc_throttle->radio_out +
				ap_motors_quard->rc_roll->pwm_out * ap_motors_quard->roll_factor[i] +
				ap_motors_quard->rc_pitch->pwm_out * ap_motors_quard->pitch_factor[i] +
				rc_yaw_constrained_pwm * ap_motors_quard->yaw_factor[i]);
    //UARTprintf("ap_motors_quard->motor_out[i]:");long_to_string(r,ap_motors_quard->motor_out[i]);UARTCharPut(UART5_BASE,r[0]);UARTCharPut(UART5_BASE,r[1]);
    //UARTCharPut(UART5_BASE,r[2]);UARTCharPut(UART5_BASE,r[3]);UARTCharPut(UART5_BASE,r[4]);UARTCharPut(UART5_BASE,r[5]);UARTprintf("\r\n");
			// calculate remaining room between fastest running motor and top of pwm range
			if( out_max - ap_motors_quard->motor_out[i] < upper_margin) {
				upper_margin = out_max - ap_motors_quard->motor_out[i];
			}
			// calculate remaining room between slowest running motor and bottom of pwm range
			if( ap_motors_quard->motor_out[i] - out_min < lower_margin ) {
				lower_margin = ap_motors_quard->motor_out[i] - out_min;
			}
		}

		// if motors are running too fast and we have enough room below, lower overall throttle
		if( upper_margin < 0 || lower_margin < 0 ) {

			// calculate throttle adjustment that equalizes upper and lower margins.  We will never push the throttle beyond this point
			motor_adjustment = (upper_margin - lower_margin) / 2;      // i.e. if overflowed by 20 on top, 30 on bottom, upper_margin = -20, lower_margin = -30.  will adjust motors -5.

			// if we have overflowed on the top, reduce but no more than to the mid point
			if( upper_margin < 0 ) {
				motor_adjustment = max(upper_margin, motor_adjustment);
			}

			// if we have underflowed on the bottom, increase throttle but no more than to the mid point
			if( lower_margin < 0 ) {
				motor_adjustment = min(-lower_margin, motor_adjustment);
			}
		}

		// move throttle up or down to to pull within tolerance
		if( motor_adjustment != 0 ) {
			for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
				ap_motors_quard->motor_out[i] += motor_adjustment;
			}

			// we haven't even been able to apply roll, pitch and minimal yaw without adjusting throttle so mark all limits as breached
			ap_motors_quard->reached_limit |= AP_MOTOR_ROLLPITCH_LIMIT | AP_MOTOR_YAW_LIMIT | AP_MOTOR_THROTTLE_LIMIT;
		}

		// if we didn't give all the yaw requested, calculate how much additional yaw we can add
		if( rc_yaw_excess != 0 ) {

			// try for everything
			yaw_to_execute = rc_yaw_excess;

			// loop through motors and reduce as necessary
			for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
				if( ap_motors_quard->yaw_factor[i] != 0 ) {

					// calculate upper and lower margins for this motor
					upper_margin = max(0, out_max - ap_motors_quard->motor_out[i]);
					lower_margin = max(0, ap_motors_quard->motor_out[i] - out_min);

					// motor is increasing, check upper limit
					if( rc_yaw_excess > 0 && ap_motors_quard->yaw_factor[i] > 0 ) {
						yaw_to_execute = min(yaw_to_execute, upper_margin);
					}

					// motor is decreasing, check lower limit
					if( rc_yaw_excess > 0 && ap_motors_quard->yaw_factor[i] < 0 ) {
						yaw_to_execute = min(yaw_to_execute, lower_margin);
					}

					// motor is decreasing, check lower limit
					if( rc_yaw_excess < 0 && ap_motors_quard->yaw_factor[i] > 0 ) {
						yaw_to_execute = max(yaw_to_execute, -lower_margin);
					}

					// motor is increasing, check upper limit
					if( rc_yaw_excess < 0 && ap_motors_quard->yaw_factor[i] < 0 ) {
						yaw_to_execute = max(yaw_to_execute, -upper_margin);
					}
				}
			}
			// check yaw_to_execute is reasonable
			if( yaw_to_execute != 0 && ((yaw_to_execute>0 && rc_yaw_excess>0) || (yaw_to_execute<0 && rc_yaw_excess<0)) ) {
				// add the additional yaw
				for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
					ap_motors_quard->motor_out[i] += (ap_motors_quard->yaw_factor[i] * yaw_to_execute);
				}
			}
			// mark yaw limit reached if we didn't get everything we asked for
			if( yaw_to_execute != rc_yaw_excess ) {
				ap_motors_quard->reached_limit |= AP_MOTOR_YAW_LIMIT;
			}
		}


		// clip motor output if required (shouldn't be)
		for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) {
			ap_motors_quard->motor_out[i] = constrain(ap_motors_quard->motor_out[i], out_min, out_max);

 	  //UARTprintf("ap_motors_quard->motor_out[i]:");long_to_string(r,ap_motors_quard->motor_out[i]);UARTCharPut(UART5_BASE,r[0]);UARTCharPut(UART5_BASE,r[1]);
    // UARTCharPut(UART5_BASE,r[2]);UARTCharPut(UART5_BASE,r[3]);UARTCharPut(UART5_BASE,r[4]);UARTCharPut(UART5_BASE,r[5]);UARTprintf("\r\n");
		}
	}
  
	// send output to each motor
	PWM_Set((unsigned long)ap_motors_quard->motor_out[0], (unsigned long)ap_motors_quard->motor_out[1], 
			(unsigned long)ap_motors_quard->motor_out[2], (unsigned long)ap_motors_quard->motor_out[3]);
}

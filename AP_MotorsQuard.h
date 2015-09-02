#ifndef __AP_MOTORS_QUARD__
#define __AP_MOTORS_QUARD__

// bit mask for recording which limits we have reached when outputting to motors
#define AP_MOTOR_NO_LIMITS_REACHED  0x00
#define AP_MOTOR_ROLLPITCH_LIMIT    0x01
#define AP_MOTOR_YAW_LIMIT          0x02
#define AP_MOTOR_THROTTLE_LIMIT     0x04
#define AP_MOTOR_ANY_LIMIT          0xFF

#define AP_MOTORS_DEFAULT_MIN_THROTTLE  130//pwm_out µÄ·¶Î§
#define AP_MOTORS_DEFAULT_MAX_THROTTLE  950
#define AP_MOTORS_MAX_NUM_MOTORS		4

#define AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM 100

#define AP_MOTORS_MATRIX_MOTOR_CW -1
#define AP_MOTORS_MATRIX_MOTOR_CCW 1

#define AP_MOTORS_MOT_1 0
#define AP_MOTORS_MOT_2 1
#define AP_MOTORS_MOT_3 2
#define AP_MOTORS_MOT_4 3
//extern unsigned long MOTOR[4];
#include "RC_Channel.h"
typedef struct _AP_MotorsQuard
{
	// bit mask to record which motor limits we hit (if any) during most recent output.  
	// Used to provide feedback to attitude controllers
	unsigned char reached_limit;

	// the minimum throttle to be sent to the engines when they're on 
	//(prevents issues with some motors on while other off at very low throttle)
	int min_throttle;
	int max_throttle;
	// input in from users
	RC_Channel* rc_roll, *rc_pitch, *rc_throttle, *rc_yaw;  
	//motor array
	int motor_out[AP_MOTORS_MAX_NUM_MOTORS];

	float roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
	float pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
	float yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
} AP_MotorsQuard;

void ap_motors_quard_init(AP_MotorsQuard *ap_motors_quard, RC_Channel *rc_roll, RC_Channel *rc_pitch, 
							RC_Channel *rc_throttle, RC_Channel *rc_yaw);
unsigned char ap_motors_quard_reached_limit(AP_MotorsQuard *ap_motors_quard, unsigned char which_limit);
void ap_motors_quard_output_armed(AP_MotorsQuard *ap_motors_quard);


#endif


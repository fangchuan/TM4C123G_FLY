#ifndef __CMD_H_
#define __CMD_H_
#include <stdint.h>
#include "stdint.h"

#include "basetype.h"
#include "math.h"
#include "protocol.h"
#include "ArduCopter.h"
#include "Parameters.h"
#include "usart.h"
#include "confige.h"
#include "PWM.h"
#include "AP_Math.h"
#include "AP_MotorsQuard.h"
void cmd_recevie_checkEvent(void);
void cmd_receiveDataAnal(uint8_t *buffer,uint8_t length);
void control_attitude_set(char buffer);
void control_on_or_off(char buffer);
void control_up_or_down(char buffer);
void Cmd_ReceiveHandler(char *buffer);


void return_hight(long h);
void return_angle(long roll,long pitch,long yaw);
void return_motor(AP_MotorsQuard*ap_motors_quard);
void exception(void);
void return_key(void);
void auto_landing(void);
void return_position(void);
void return_target_angle(void);

#endif

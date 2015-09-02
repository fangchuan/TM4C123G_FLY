/*********************************
 * Sys.h
 *
 *  Created on: 2014-6-29
 *      Author: Administrator
 ********************************/
#ifndef __SYS_H__
#define __SYS_H__

#include <stdint.h>
#include <stdbool.h>
#include "TM4C.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
extern unsigned long  sys_millisecond ;
void ClockInit(void);
void Delay_us(unsigned long i);
void Delay_ms(unsigned long i);
unsigned long Get_ms(void);

#endif /* SYS_H_ */

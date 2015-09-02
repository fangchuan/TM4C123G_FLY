#include "TM4C.h"
#include "Delay.h"
void delay_millis(unsigned long millis)
{
	unsigned long us=millis;
	ROM_SysCtlDelay(80/3*us);
}

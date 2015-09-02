#include "Sys.h"
unsigned long  sys_millisecond ;	//run time in ms

void SysTickIntHandler(){
	sys_millisecond ++;
}

void ClockInit(){
	IntMasterEnable();
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	//	200M/4
	SysTickEnable();
	SysTickIntEnable();
	SysTickPeriodSet(SysCtlClockGet()/1000-1);	//period 1ms
	sys_millisecond  = 0;
	SysTickIntRegister(SysTickIntHandler);
}

unsigned long  Get_ms(){
	return sys_millisecond ;
}

void Delay_us(unsigned long i){
	for(;i>0;i--){
		SysCtlDelay(SysCtlClockGet()/3/1000000);
	}
}

void Delay_ms(unsigned long i){
	for(; i>0; i--){
		Delay_us(1000);
	}
}


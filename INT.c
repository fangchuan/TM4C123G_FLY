#include "stdint.h"
#include "stdbool.h"
#include "common.h"
#include "TM4C.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "Timer_user.h"
#include "Parameters.h"
#include "ArduCopter.h"
#include "INT.h"
RC_S RC;
uint32_t pulse_start[4] = {0};
uint32_t pulse_end[4] = {0};
uint32_t pulse_flag[4] = {0};
uint16_t pulse_buffer[4][9] = {0};
#define FILTER_DEPTH	8
#define FILTER_THRESHOLD	100
void PortAIntHandler(){
				uint16_t avg=0;
				uint8_t i;
	  uint32_t status = GPIOPinIntStatus(GPIO_PORTA_BASE, false);

	 if(status | GPIO_PIN_3){								//Pulse get
		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3)){		//rising edge
			//TimerLoadSet(TIMER0_BASE, TIMER_BOTH, SYSCLK);
			if(pulse_flag[0] == 0){
				pulse_start[0] = TimerValueGet(TIMER0_BASE, TIMER_A);
				//pulse_time[0] = Get_ms();
				pulse_flag[0] = 1;
			}
			//DEBUG_PRINTLN("\nhigh\n");
		}else{
			pulse_end[0] = TimerValueGet(TIMER0_BASE, TIMER_A);
			//uint32_t pulse = TimerValueGet(TIMER0_BASE, TIMER_A);
			if(pulse_flag[0] == 1){
				//pulse = (SYSCLK-pulse+25)/50;
				int16_t pulse = (pulse_start[0] - pulse_end[0])/(80000000/1000000);		//unit is us
				while(pulse <= 950){
					pulse = 1000 + pulse;
					//DEBUG_PRINTLN("F4 neg\n");
				}
	
				pulse_flag[0] = 0;


				for(i=0; i<FILTER_DEPTH; i++){
					avg += pulse_buffer[0][i];
					pulse_buffer[0][i] = pulse_buffer[0][i+1];
				}
				if(pulse < 900){
					pulse_buffer[0][FILTER_DEPTH] = 1000;
				}else if(pulse > 2100){
					pulse_buffer[0][FILTER_DEPTH] = 2000;
				}else{
					pulse_buffer[0][FILTER_DEPTH] = pulse;
				}
				avg = avg / FILTER_DEPTH;
				if(abs(avg-pulse) > FILTER_THRESHOLD){
					//DEBUG_PRINTLN("avg=%d pulse=%d\n", avg, pulse);
					pulse = avg;
				}

				if((pulse > 950)&&(pulse < 2050)){
					RC.ch3 = pulse;
				}else{
					RC.ch3 = 1000;
				}
			}
		}
		GPIOPinIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
	}

}
void ReceiveInit(){
	uint16_t i,j;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOInput( GPIO_PORTA_BASE, GPIO_PIN_3);		

	//GPIOIntRegister(GPIO_PORTA_BASE, PortAIntHandler);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_BOTH_EDGES);

	GPIOPinIntEnable( GPIO_PORTA_BASE, GPIO_PIN_3);
	
	for(i=0; i<4; i++){
		for(j=0; j<9; j++){
			pulse_buffer[i][j] = 1500;
		}
	}
}
// 	if(status | GPIO_PIN_3){								//A3 Pulse get
// 		if(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3)){		//rising edge
// 			//TimerLoadSet(TIMER0_BASE, TIMER_BOTH, SYSCLK);
// 			if(pulse_flag[2] == 0){
// 				pulse_start[2] = TimerValueGet(TIMER0_BASE, TIMER_A);
// 				//pulse_time[2] = Get_ms();
// 				pulse_flag[2] = 1;
// 			}
// 			//DEBUG_PRINTLN("\nhigh\n");
// 		}else{
// 			pulse_end[2] = TimerValueGet(TIMER0_BASE, TIMER_A);
// 			//uint32_t pulse = TimerValueGet(TIMER0_BASE, TIMER_A);
// 			if(pulse_flag[2] == 1){

// 				int16_t pulse = (pulse_start[2] - pulse_end[2])/(SYSCLK/1000000);		//unit is us
// 				while(pulse <= 950){
// 					pulse = 1000 + pulse;
// 			
// 				}
// 			
// 				pulse_flag[2] = 0;

// 				uint16_t avg=0;
// 				uint8_t i;
// 				for(i=0; i<FILTER_DEPTH; i++){
// 					avg += pulse_buffer[2][i];
// 					pulse_buffer[2][i] = pulse_buffer[2][i+1];
// 				}
// 				if(pulse < 900){
// 					pulse_buffer[2][FILTER_DEPTH] = 1000;
// 				}else if(pulse > 2100){
// 					pulse_buffer[2][FILTER_DEPTH] = 2000;
// 				}else{
// 					pulse_buffer[2][FILTER_DEPTH] = pulse;
// 				}
// 				avg = avg / FILTER_DEPTH;
// 				if(abs(avg-pulse) > FILTER_THRESHOLD){
// 					//DEBUG_PRINTLN("avg=%d pulse=%d\n", avg, pulse);
// 					pulse = avg;
// 				}

// 				if((pulse > 950)&&(pulse < 2050)){
// 					RC.ch3 = pulse;
// 				}else{
// 					RC.ch3 = 1000;
// 				}
// 			}
// 		}
// 		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
// 	}

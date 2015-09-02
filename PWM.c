
//*****************************************************************************
//B4--PWM1,B5--PWM2,C4--PWM3,C5--PWM4
//***************************************************************************8
#include "TM4C.h"
#include "PWM.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
//unsigned long PWM=0;
//int jiasu=0,jiansu=0;
static unsigned long PWM1=0;
static unsigned long PWM2=0;
static unsigned long PWM3=0;
static unsigned long PWM4=0;
void PWM_Init(void)
{
	  
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);	   // 
	  SysCtlPWMClockSet(SYSCTL_PWMDIV_32);//PWM Clock=80M/32
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);   // 
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);             //
		GPIOPinConfigure(GPIO_PB5_M0PWM3);
	  GPIOPinConfigure(GPIO_PC4_M0PWM6);
	  GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);    // 
		GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
	  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
	  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);
    PWMGenConfigure(PWM_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);		     // 
	  PWMGenConfigure(PWM_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, 50000);   // ????PWM??????N = (1 / f) * SysClk.  ???:131071
	  PWMGenPeriodSet(PWM_BASE, PWM_GEN_3, 50000);//Load=pwmclock/frequence, 50HZ
//    PWMPulseWidthSet(PWM_BASE, PWM_OUT_2,5000);	 
// 		PWMPulseWidthSet(PWM_BASE, PWM_OUT_3,5000);   
// 		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6,5000);
// 		PWMPulseWidthSet(PWM_BASE, PWM_OUT_7,5000);
    PWMOutputState(PWM_BASE, PWM_OUT_2_BIT| PWM_OUT_3_BIT|PWM_OUT_6_BIT|PWM_OUT_7_BIT, true); //
    PWMGenEnable(PWM_BASE, PWM_GEN_1);	            //
		PWMGenEnable(PWM_BASE, PWM_GEN_3);
		
		//ROM_SysCtlDelay(5000*(SysCtlClockGet()/3000));
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_2,2500);	 
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_3,2500);   
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6,2500);
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_7,2500);
		//ROM_SysCtlDelay(5000*(SysCtlClockGet()/3000));
}

void PWM_Set(unsigned long motor1,unsigned long motor2,unsigned long motor3,unsigned long motor4)
{ 

	 	PWMPulseWidthSet(PWM_BASE, PWM_OUT_2,(motor1-1230)*3.37+2500);	 
  	PWMPulseWidthSet(PWM_BASE, PWM_OUT_3,(motor2-1230)*3.37+2500);   
  	PWMPulseWidthSet(PWM_BASE, PWM_OUT_6,(motor3-1230)*3.37+2500);
  	PWMPulseWidthSet(PWM_BASE, PWM_OUT_7,(motor4-1230)*3.37+2500);

}


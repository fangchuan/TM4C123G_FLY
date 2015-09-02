#include "stdint.h"
#include "stdbool.h"
#include "common.h"
#include "MPU6050.h"
#include "UART_user.h"
#include "TM4C.h"
#include "mpu_tool.h"
#include "inv_mpu.h"
#include "Parameters.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "Timer_user.h"
#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_AHRS_DCM.h"
#include "AP_Math.h"
#include "ArduCopter.h"
#include "RC_Channel.h"
#include "AP_MotorsQuard.h"
#include "FC_PID.h"
#include "confige.h"
#include "PWM.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "string.h"
#include "math.h"
#include "NRF.h"
#include "Delay.h"
#include "ccdandcsb.h"
#include "INT.h"
#include "cmd.h"
#include "AP_OpticalFlow_ADNS3080.h"
#include "AP_OpticalFlow.h"

void reg_int_handler(void);
void Control(void);

void reg_int_handler(void)
{
    unsigned long ulStatus;//, ulValue;
    
    ulStatus = GPIOPinIntStatus(reg_int_param.gpio_port, false);            //  ��ȡ�ж�״̬
    GPIOPinIntClear(reg_int_param.gpio_port, ulStatus);                    //  ����ж�״̬����Ҫ    
    (reg_int_param.cb)();

}
extern _Q_ANGLE  Q_ANGLE;
extern _Q_ANGLE  Q_AngleVelocity;
extern char BUFFER[8];

extern long height_current;									//ȡ�ó���������ֵ

/********************************************************������ģ��***************************************************************************************/
void UART5Intcsb(void)
{
    unsigned long ulStatus;
    uint8_t high8,low8;
    ulStatus = ROM_UARTIntStatus(UART5_BASE, true);    // ��ȡ�ж�״̬
    ROM_UARTIntClear(UART5_BASE, ulStatus);    // ����жα�־
    while(ROM_UARTCharsAvail(UART5_BASE))    // ȷ���Ƿ����κ��ַ�����FIFO
    {
    	high8 = ROM_UARTCharGetNonBlocking(UART5_BASE);   //�������ݸ�8λ
    	low8 = ROM_UARTCharGetNonBlocking(UART5_BASE);   //�������ݵ�8λ
    	height_current=(high8*256+low8);//��mmΪ��λ
    }
}
int main(void)
{
     FPUEnable();
     FPULazyStackingEnable();
//The hardware priority mechanism will only look at the upper 3 bits of the priority level, so
//any prioritization must be performed in those bits
// 	  ROM_IntPrioritySet(INT_UART1,0<<5);
//    ROM_IntPrioritySet(INT_GPIOF,1<<5);
// 	  ROM_IntPrioritySet(INT_UART5,2<<5);
// 	  ROM_IntPrioritySet(INT_TIMER0A,3<<5);
    ClockInit();
	  intCHAOSHENGBO();
    //InitCSB_IO();
  	InitConsole0();
	  SPI_Init();
	  PWM_Init();
    I2C_Init();
    if(!(dmp_init()))
		while(1);
    Self_Test();
		
		arducopter_init();
		
	  Timer01Init();
	
    while(1)
    {	
      Cmd_ReceiveHandler(BUFFER);
		  if(begin_fly==1)
	    {
            Control();
      }
      //CSB_FS();
		  ROM_UARTCharPut(UART5_BASE,0X55);//��������ʼ�ź�
    }
    
}

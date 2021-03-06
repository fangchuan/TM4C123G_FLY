#include "TM4C.h"
#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_AHRS_DCM.h"
#include "AP_Math.h"
#include "ArduCopter.h"
#include "RC_Channel.h"
#include "AP_MotorsQuard.h"
#include "Timer_user.h"
#include "cmd.h"
#include "MPU6050.h"
#include "Parameters.h"
#include "AP_AHRS_DCM.h"
#include "debug.h"
extern _Q_ANGLE Q_ANGLE;
extern AP_MotorsQuard ap_motors_quard;
extern long height_current;
extern AP_AHRS_DCM ap_ahrs_dcm;


void Timer01Init()  //定时器0、1的使能
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);   //	使能外设
    
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);// 配置2个32位周期定时器。
   //中断频率为Load/system clock
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/5);// 配置2个32位周期定时器的装载值。
  	ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 800000);
    ROM_IntEnable(INT_TIMER0A);
    ROM_IntEnable(INT_TIMER1A);
	 
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // 设置定时器超时中断�
  	ROM_IntMasterEnable();// 开总中断
	
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    //ROM_TimerEnable(TIMER1_BASE, TIMER_A);  // 使能定时器0和1
}
void Timer01Stop()  //定时器0、1的禁用
{
	ROM_TimerDisable(TIMER0_BASE,TIMER_A);
  //ROM_TimerDisable(TIMER1_BASE,TIMER_A);
}
void Timer01Start()
{
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
	//ROM_TimerEnable(TIMER1_BASE, TIMER_A);
}

// //-----------------------------------------------------------------------------
// // 定时器0中断
// //-----------------------------------------------------------------------------
extern int auto_land;
void Timer0IntHandler(void)
{
    static int s=1;
	  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);    // 清除时钟中断
	  exception();
	  //return_hight(height_current);
	  switch(s)
   	{
			case 1:return_angle(ap_ahrs_dcm.roll_sensor,ap_ahrs_dcm.pitch_sensor,ap_ahrs_dcm.yaw_sensor);break;
			case 2:return_motor(&ap_motors_quard);break;
			case 3:return_hight(height_current);break;
			case 4:return_position();break;
			//case 5:return_target_angle();break;
    }
		s++;
		if(s > 5) s=1;
   
}
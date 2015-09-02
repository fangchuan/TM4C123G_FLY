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


void Timer01Init()  //¶¨Ê±Æ÷0¡¢1µÄÊ¹ÄÜ
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);   //	Ê¹ÄÜÍâÉè
    
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);// ÅäÖÃ2¸ö32Î»ÖÜÆÚ¶¨Ê±Æ÷¡£
   //ÖÐ¶ÏÆµÂÊÎªLoad/system clock
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/5);// ÅäÖÃ2¸ö32Î»ÖÜÆÚ¶¨Ê±Æ÷µÄ×°ÔØÖµ¡£
  	ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 800000);
    ROM_IntEnable(INT_TIMER0A);
    ROM_IntEnable(INT_TIMER1A);
	 
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // ÉèÖÃ¶¨Ê±Æ÷³¬Ê±ÖÐ¶Ï¡
  	ROM_IntMasterEnable();// ¿ª×ÜÖÐ¶Ï
	
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    //ROM_TimerEnable(TIMER1_BASE, TIMER_A);  // Ê¹ÄÜ¶¨Ê±Æ÷0ºÍ1
}
void Timer01Stop()  //¶¨Ê±Æ÷0¡¢1µÄ½ûÓÃ
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
// // ¶¨Ê±Æ÷0ÖÐ¶Ï
// //-----------------------------------------------------------------------------
extern int auto_land;
void Timer0IntHandler(void)
{
    static int s=1;
	  ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);    // Çå³ýÊ±ÖÓÖÐ¶Ï
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
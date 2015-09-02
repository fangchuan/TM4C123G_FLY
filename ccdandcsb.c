#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "ccdandcsb.h"
#include "Sys.h"
#include "ArduCopter.h"
#include "AP_OpticalFlow_ADNS3080.h"
/**************超声波和摄像头***********************

注：时钟80mhz	超声波VCC必须接5V电压否则不工作    两个模块工作总时间[9ms,25ms]随超声波时间而定
io口  
调用两个子函数即可获得距离(cm)

      摄像头
io口  PB6-------SI
	PE2-------AO
	PB7-------CLK

*********************************************/


void intCHAOSHENGBO(void);					//超声波初始化
										//注只需调用intCHAOSHENGBO和CHAOSHENGBO函数即可


int adc1(void);							//摄像头ad采集
void intadc(void);							//摄像头ad初始化
void intccd(void);							//摄像头初始化
void caiji(void);							//摄像头采集
int ccd(void);							    //摄像头采集及处理    返回	{100 ：右转		10：直行		1：左转	0：没看清楚}
										//注只需调用intccd和ccd函数即可

//摄像头所需外部变量
uint32_t n[128];
uint32_t adcbase=0;                    //ADC基准
uint32_t i=0;
uint32_t ccd_a=0,ccd_b=0,ccd_c=0;

unsigned int ui8LED = 2;
char ccd_direction;
int led=0;
//超声波所需外部变量


void intCHAOSHENGBO(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinConfigure(GPIO_PE4_U5RX);
	GPIOPinConfigure(GPIO_PE5_U5TX);												//使能GPIO模块
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);  // 
	UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(),9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));   // 配置UART到9600,8-N-0
	IntEnable(INT_UART5);    // 使能UART中断
	UARTIntEnable(UART5_BASE,UART_INT_RX);
	IntMasterEnable(); // 使能处理器总中断

}
/********************************************************超声波模块end***************************************************************************************/
void InitCSB_IO(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_GPIOPinTypeGPIOOutput( GPIO_PORTE_BASE, GPIO_PIN_4);
	ROM_GPIOPinTypeGPIOInput( GPIO_PORTE_BASE, GPIO_PIN_5);
	ROM_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_HIGH_LEVEL);
	GPIOPinIntEnable( GPIO_PORTE_BASE, GPIO_PIN_5);
	IntEnable(INT_GPIOE); // 使能GPIOe端口中断
	IntMasterEnable(); // 使能处理器总中断
}
void GPIO_INT_E(void)
 {
	 static int s=0;
   unsigned long current_time,pre_time;
   unsigned long ulStatus;
   ulStatus = ROM_GPIOPinIntStatus(GPIO_PORTE_BASE, true); // 读取中断状态
   ROM_GPIOPinIntClear(GPIO_PORTE_BASE, ulStatus); // 清除中断状态，重要
   if (ulStatus & GPIO_PIN_5) // 如果KEY的中断状态有效
	 {
		 s++;
	   current_time = Get_ms();
	}
	if( s == 2)
		pre_time = current_time;
	height_current = (current_time - pre_time)*0.34*0.5;
 }
void CSB_FS(void)
{
			ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4,GPIO_PIN_4 );
			inerDelay_us(15);
			ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4,0 );
}


/********************************************************摄像头模块***************************************************************************************/
void intadc()
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);//adc0外设初始化
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//E2,

		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);//E2ADC输入

		ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);//配置ADC0           采样序列3  ADC处理器触发    优先级为0

		ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
		//配置ADC采样序列发生的步进         ADC模块基址ADC_BASE adc采样序列的编号（0,1,2,3）   步值  步值的配置
}

int adc1()
{
	uint32_t num[1];
	ADCSequenceEnable(ADC0_BASE, 3);//采样
	ADCIntClear(ADC0_BASE, 3);//中断清除
	ADCProcessorTrigger(ADC0_BASE, 3);//触发ADC转换
	while(!ADCIntStatus(ADC0_BASE, 3, false));//等待转换完成
	ADCSequenceDataGet(ADC0_BASE, 3, num);//读取ADC值
	return(num[0]);
}

void intccd()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6|GPIO_PIN_7);//
	intadc();
}

void caiji()
{
		adcbase=0;
		for(i=0;i<=127;i++)
		{
			n[i]=0;
		}
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6,GPIO_PIN_6 );//SI=1
	for(i=0;i<=127;i++)
		{
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7,GPIO_PIN_7 );//CLK==1
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6,0 );//SI=0
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7,0 );//clk=0
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6,0 );//
			n[i]=adc1();
		}
}

int ccd()
{


	caiji();
	/***************************信号处理*******************************/
	for(i=0;i<=127;i++)
	{
		n[i]=(n[i]>=4080)?0:n[i];
		adcbase=adcbase+n[i];
	}

	adcbase=adcbase*6/5/128;											//adc基准电压以后再改**************************************

	for(i=0;i<=127;i++)
	{
		n[i]=(n[i]>=adcbase)?0:1;
	}

	for(i=2;i<=125;i++)   					//滤波
	{
		if(n[i])
			{if(n[i-1]+n[i-2]>1||n[i+2]+n[i+1]>1);
			else n[i]=0;}
		else
			{if(n[i-1]+n[i-2]<1||n[i+2]+n[i+1]<1);
			else n[i]=1;}
	}

	i=0;
	while(n[i]==1)
	{
		n[i]=0;
		i++;
	}
	i=127;
	while(n[i]==1)
	{
		n[i]=0;
		i--;
	}
	ccd_a=0;
	ccd_b=0;
	ccd_c=0;
	for(i=0;i<31;i++)
	{
		ccd_a=ccd_a+n[i+16];
		ccd_c=ccd_c+n[i+80];
		ccd_b=ccd_b+n[i+48];
	}
	if(ccd_b>ccd_a+ccd_c)
		return(10);
	else
		if(ccd_c>ccd_a+ccd_b)
			return(1);
		else
			if(ccd_a>ccd_b+ccd_c)
				return(100);
			else
			{
				if(ccd_b>(ccd_a+ccd_c+ccd_b)/3)
				return(10);
			else
				if(ccd_c>(ccd_a+ccd_b+ccd_c)/3&&ccd_c>ccd_a)
					return(1);
				else
					if(ccd_a>(ccd_a+ccd_b+ccd_c)/3&&ccd_a>ccd_c)
						return(100);
					else return(0);
			}
	/******************************信号处理end************************************************/
}





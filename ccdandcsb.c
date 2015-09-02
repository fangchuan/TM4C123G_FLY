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
/**************������������ͷ***********************

ע��ʱ��80mhz	������VCC�����5V��ѹ���򲻹���    ����ģ�鹤����ʱ��[9ms,25ms]�泬����ʱ�����
io��  
���������Ӻ������ɻ�þ���(cm)

      ����ͷ
io��  PB6-------SI
	PE2-------AO
	PB7-------CLK

*********************************************/


void intCHAOSHENGBO(void);					//��������ʼ��
										//עֻ�����intCHAOSHENGBO��CHAOSHENGBO��������


int adc1(void);							//����ͷad�ɼ�
void intadc(void);							//����ͷad��ʼ��
void intccd(void);							//����ͷ��ʼ��
void caiji(void);							//����ͷ�ɼ�
int ccd(void);							    //����ͷ�ɼ�������    ����	{100 ����ת		10��ֱ��		1����ת	0��û�����}
										//עֻ�����intccd��ccd��������

//����ͷ�����ⲿ����
uint32_t n[128];
uint32_t adcbase=0;                    //ADC��׼
uint32_t i=0;
uint32_t ccd_a=0,ccd_b=0,ccd_c=0;

unsigned int ui8LED = 2;
char ccd_direction;
int led=0;
//�����������ⲿ����


void intCHAOSHENGBO(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinConfigure(GPIO_PE4_U5RX);
	GPIOPinConfigure(GPIO_PE5_U5TX);												//ʹ��GPIOģ��
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);  // 
	UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(),9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));   // ����UART��9600,8-N-0
	IntEnable(INT_UART5);    // ʹ��UART�ж�
	UARTIntEnable(UART5_BASE,UART_INT_RX);
	IntMasterEnable(); // ʹ�ܴ��������ж�

}
/********************************************************������ģ��end***************************************************************************************/
void InitCSB_IO(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_GPIOPinTypeGPIOOutput( GPIO_PORTE_BASE, GPIO_PIN_4);
	ROM_GPIOPinTypeGPIOInput( GPIO_PORTE_BASE, GPIO_PIN_5);
	ROM_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_HIGH_LEVEL);
	GPIOPinIntEnable( GPIO_PORTE_BASE, GPIO_PIN_5);
	IntEnable(INT_GPIOE); // ʹ��GPIOe�˿��ж�
	IntMasterEnable(); // ʹ�ܴ��������ж�
}
void GPIO_INT_E(void)
 {
	 static int s=0;
   unsigned long current_time,pre_time;
   unsigned long ulStatus;
   ulStatus = ROM_GPIOPinIntStatus(GPIO_PORTE_BASE, true); // ��ȡ�ж�״̬
   ROM_GPIOPinIntClear(GPIO_PORTE_BASE, ulStatus); // ����ж�״̬����Ҫ
   if (ulStatus & GPIO_PIN_5) // ���KEY���ж�״̬��Ч
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


/********************************************************����ͷģ��***************************************************************************************/
void intadc()
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);//adc0�����ʼ��
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//E2,

		GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);//E2ADC����

		ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);//����ADC0           ��������3  ADC����������    ���ȼ�Ϊ0

		ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
		//����ADC�������з����Ĳ���         ADCģ���ַADC_BASE adc�������еı�ţ�0,1,2,3��   ��ֵ  ��ֵ������
}

int adc1()
{
	uint32_t num[1];
	ADCSequenceEnable(ADC0_BASE, 3);//����
	ADCIntClear(ADC0_BASE, 3);//�ж����
	ADCProcessorTrigger(ADC0_BASE, 3);//����ADCת��
	while(!ADCIntStatus(ADC0_BASE, 3, false));//�ȴ�ת�����
	ADCSequenceDataGet(ADC0_BASE, 3, num);//��ȡADCֵ
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
	/***************************�źŴ���*******************************/
	for(i=0;i<=127;i++)
	{
		n[i]=(n[i]>=4080)?0:n[i];
		adcbase=adcbase+n[i];
	}

	adcbase=adcbase*6/5/128;											//adc��׼��ѹ�Ժ��ٸ�**************************************

	for(i=0;i<=127;i++)
	{
		n[i]=(n[i]>=adcbase)?0:1;
	}

	for(i=2;i<=125;i++)   					//�˲�
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
	/******************************�źŴ���end************************************************/
}





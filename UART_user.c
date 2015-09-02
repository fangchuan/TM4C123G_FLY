#include "UART_user.h"
#include "cmd.h"
#include "string.h"
char receive_buffer[8]={0};
char BUFFER[8]={0};
int sign=0;
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
//-----------------------------------------------------------------------------
void
InitConsole0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);	//
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);				//
    GPIOPinConfigure(GPIO_PA1_U0TX);				//
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//
    //UARTStdioInit(0);	   //
	  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
		IntEnable(INT_UART0);    // ʹ��UART�ж�
	  UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_TX);
	  IntMasterEnable(); // ʹ�ܴ��������ж�
}
//---------------------------------------------------
//UART5��ʼ��   
//---------------------------------------------------
void
InitConsole1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	// ʹ��GPIO����
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    GPIOPinConfigure(GPIO_PB0_U1RX);				//	���ô��ڽ��չܽ�
    GPIOPinConfigure(GPIO_PB1_U1TX);				//	 ���ô��ڷ��͹ܽ�
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//��GPIO���ܸ���Ϊ����
	  UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));   // ����UART��115200,8-N-0
    //UARTStdioInit(1);	   //���ô�������
   // UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);//default work as 1 byte deep
	  IntEnable(INT_UART1);    // ʹ��UART�ж�
	  UARTIntEnable(UART1_BASE,UART_INT_RX|UART_INT_TX);
	  IntMasterEnable(); // ʹ�ܴ��������ж�
}
int count=0;
int count_trans=0;
int  UART0IntHandler(void)
{
    uint32_t ui32Status;
		static unsigned char bled;
		bled++;

    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    ROM_UARTIntClear(UART0_BASE, ui32Status);
	  while(ROM_UARTCharsAvail(UART0_BASE))
		{
			receive_buffer[count] = UARTCharGetNonBlocking(UART0_BASE); 
			if(count==0&&receive_buffer[0]!=0x46) return 0;
			if(count==1&&receive_buffer[1]!=0x41) return 0;
			count++;
			if(count==8)
			{
				memcpy(BUFFER,receive_buffer,8);
				count=0;
				sign=1;
				count_trans++;
      }
			
		}
		
		if(bled==1)	
			GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, BLUE_LED);
		if(bled>=2)
		{
			bled=0;
			GPIOPinWrite(GPIO_PORTF_BASE, BLUE_LED, 0);
		}
}
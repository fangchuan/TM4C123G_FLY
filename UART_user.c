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
		IntEnable(INT_UART0);    // 使能UART中断
	  UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_TX);
	  IntMasterEnable(); // 使能处理器总中断
}
//---------------------------------------------------
//UART5初始化   
//---------------------------------------------------
void
InitConsole1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	// 使能GPIO外设
	  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    GPIOPinConfigure(GPIO_PB0_U1RX);				//	配置串口接收管脚
    GPIOPinConfigure(GPIO_PB1_U1TX);				//	 配置串口发送管脚
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//把GPIO功能复用为串口
	  UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));   // 配置UART到115200,8-N-0
    //UARTStdioInit(1);	   //设置串口属性
   // UARTFIFOLevelSet(UART1_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);//default work as 1 byte deep
	  IntEnable(INT_UART1);    // 使能UART中断
	  UARTIntEnable(UART1_BASE,UART_INT_RX|UART_INT_TX);
	  IntMasterEnable(); // 使能处理器总中断
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
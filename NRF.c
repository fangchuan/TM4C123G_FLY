#include "stdint.h"
#include "stdbool.h"
#include "inc/hw_types.h"		   //??????
#include "inc/hw_memmap.h"		   //????????
#include "string.h"
#include "driverlib/sysctl.h" 		   //
#include "driverlib/gpio.h"   		   //
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"		   //
#include "driverlib/fpu.h"			   //
#include "inc/hw_ints.h"			  //???????
#include "driverlib/interrupt.h"	  //
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "NRF.h"
#include "ccdandcsb.h"
#include "ArduCopter.h"
char  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//±¾µØµØÖ·
char  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//½ÓÊÕµØÖ·
char  sta;
char RxBuf[32]={0};
//RC_GETDATA NRF_GET;
//void Rec_Data_Init()
//{
//	NRF_GET.START=0;
//  NRF_GET.STOP=0;
//	NRF_GET.THROTTLE=0;
//}
//void Rec_Data_Pro(char RxBuf)//½ÓÊÕµ½µÄÊý¾Ý½øÐÐ´¦Àí
//{
//	if(nRF24L01_RxPacket(&RxBuf)) 
//	{
//		if(RxBuf==0x01) NRF_GET.START=1;
//		if(RxBuf==0x02) NRF_GET.STOP=1;
//		if(RxBuf==0x03) NRF_GET.THROTTLE=1;
//		//if(RxBuf==0x04) NRF_GET.THROTTLE=1250;//ÔÝÊ±Ã»ÓÃµ½
//		//if(RxBuf==0x05) NRF_GET.THROTTLE=1500;
//		//if(RxBuf==0x06) NRF_GET.THROTTLE=1750;
//		//if(RxBuf==0x07) NRF_GET.THROTTLE=2000;
//		
//}
//}
void  IfReciveNRF()
{//
	     
			 SetRX_Mode();    //×ª»»Îª½ÓÊÕÄ£Ê½
			 if(nRF24L01_RxPacket(RxBuf))   //Ò»µ©½ÓÊÕµ½Êý¾Ý
        {
	        handle_command(RxBuf[0]);
	       SPI_RW_Reg(WRITE_REG+STATUS,0XFF);
					//RxBuf=0;
        }
        //ms_delay();	
}
void GPIO_intInIRQ(void)
{
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);							//Ê¹ÄÜGPIOÄ£¿é²»ÐèÒªÔÚ½ÓÊÕÄ£¿éÖÐÒÑ¾­Ê¹ÄÜ¹ýÒÔºó¿ÉÒÔ¿´Çé¿ö
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);							//Ð¡µÆio¿ÚÊ±ÄÜ
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);				//Ê¹ÄÜGPIO¶Ë¿Ú
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);						//Ð¡µÆÐ´0

//	GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);		//ÅäÖÃ°´¼ü¶Ë¿Ú
//	GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN);							//ÉèÖÃ¶Ë¿Ú·½Ïò
	GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_LOW_LEVEL);								//µÍµçÆ½ÖÐ¶Ï  ¾ö¶¨Ê²Ã´Ê±ºò½øÈëÖÐ¶Ï
	GPIOPinIntEnable(GPIO_PORTB_BASE, GPIO_PIN_1);												//Ê¹ÄÜÖÐ¶ÏÄ£¿é
	IntEnable(INT_GPIOB);															//Ê¹ÄÜGPIO¶Ë¿ÚÖÐ¶Ï
	IntMasterEnable();																		//¿ª×ÜÖÐ¶Ï********************************×¢ÒâÒÔºó»¹ÓÐÖÐ¶Ï¾ÍÒª¸Ä
}
//**********************************************************½ÓÊÕÖÐ¶Ïº¯Êý****************************************//
void GPIO_INT_B(void)
 {
	if (GPIOPinIntStatus(GPIO_PORTB_BASE, true)& GPIO_PIN_1) 													// Èç¹û½ÓÊÕµÄÖÐ¶Ï×´Ì¬ÓÐÐ§
	   {
	   /******************************************°´¼üºóÖÐ¶ÏÊµÏÖ·¢ËÍ	¿ÉÒÔ¼Ó³ÌÐò*****************************/

			if(nRF24L01_RxPacket(RxBuf))   //Ò»µ©½ÓÊÕµ½Êý¾Ý
        {
	        handle_command(RxBuf[0]);

        }		SetRX_Mode();

	   /******************************************°´¼üºóÖÐ¶ÏÊµÏÖ·¢ËÍ*****************************/
	   }
	GPIOPinIntClear(GPIO_PORTB_BASE,GPIOPinIntStatus(GPIO_PORTB_BASE, true));//·µ»Ø´ËÊ±µÄÖÐ¶Ï×´Ì¬²¢Çå³ý
 }  

void NRF_Init()
{
	RF24L01_IO_set(); //NRFÓÃÓÚÄ£ÄâSPIµÄÒý½ÅÉèÖÃ   
	init_NRF24L01() ;//NRF³õÊ¼»¯£¬2M,ÆµµÀ0£ £¬,2.4G £¬Ö÷½ÓÊÜ
  GPIO_intInIRQ();
	SetRX_Mode();    //×ª»»Îª½ÓÊÕÄ£Ê½

}
//===========================RF24L01????==========================================
void RF24L01_IO_set(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);//CS--B0
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);//E1--MOSI A6--CLK  A7--CE		
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7);//E1--MOSI A6--CLK  A7--CEÒÔÇ°ÊÇE4E5
	ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
	ROM_GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
	ROM_GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_6|GPIO_PIN_7,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
  ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_1,GPIO_DIR_MODE_IN);//B1--IRQ
	ROM_GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_0,GPIO_DIR_MODE_IN);//E0--MISO
	ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_OD_WPU);
	ROM_GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_0,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_OD_WPU);
}
void IO_Init(unsigned char x)
{
	 switch(x)
   {
		case 0x00:break;
	 case 0x01:	//CE_1,CE_0
	 ROM_GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_PIN_7);break;
	 case 0x02:
	 ROM_GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_7,0);break;
	 case 0x03:  //CS_1,CS_0
	 ROM_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_PIN_0);break;
	 case 0x04: 
	 ROM_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,0);break;	
   case 0x05:  //clk_1,clk_0
	 ROM_GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,GPIO_PIN_6);break;		 
	 case 0x06:
	 ROM_GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6,0);break;	
   case 0x07:  //MOSI_1,MOSI_0
	 ROM_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1);break;	
   case 0x08:
	 ROM_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0);break;
   case 0x09:  //MISO_1,MISO_0
	 ROM_GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_0,GPIO_DIR_MODE_IN);
	 break;	
   case 0x10:
	 ROM_GPIODirModeSet(GPIO_PORTE_BASE, GPIO_PIN_0,GPIO_DIR_MODE_IN);
	 break;	
   case 0x11:  //IRQ_1,IRQ_0
	 ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_1,GPIO_DIR_MODE_IN);
	 break;
	 case 0x12:
	 ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_1,GPIO_DIR_MODE_IN);
	 break;
  default:break;
  }
}

//========================delay 2ms=============================================
void ms_delay(void)
{ 
   ROM_SysCtlDelay(2 * (SysCtlClockGet()/3000));
}
//×¢Òâ£¡£¡£¡£¡£¬ÏµÍ³ÆµÂÊÒ»µ©¸Ä±ä£¬Õâ¸öº¯ÊýÒ²Òª¸Ä£¡£¡£¡
void inerDelay_us(int n)
{
   ROM_SysCtlDelay(80/3*n);
}


//========================================???================================
void Delay(int s)
{
   ROM_SysCtlDelay(100 *s* (SysCtlClockGet()/3000));
}
//==============================================================================
//??:uint SPI_RW(uint uchar)
//??:NRF24L01µÄSPIÐ´Ê±Ðò
//******************************************************************************
char SPI_RW(char data)
{	
  char i,temp=0;
	RF24L01_SCK_0; 
  for(i=0;i<8;i++) // output 8-bit
   	{
	if((data & 0x80)==0x80)
	{
		RF24L01_MOSI_1;         // output 'uchar', MSB to MOSI
	}
	else
	{
	 	RF24L01_MOSI_0; 
	}	
		data = (data << 1);            // shift next bit into MSB..
		temp = (temp << 1);
		RF24L01_SCK_1;                // Set SCK high..
		if((ROM_GPIOPinRead(GPIO_PORTE_BASE,GPIO_PIN_0)&0X01))temp++;         // capture current MISO bit
	  
		RF24L01_SCK_0;              // ..then set SCK low again
   	}
    return(temp);           		  // return read uchar  		
}
//****************************************************************************************************
//??:uchar SPI_Read(uchar reg)
//??:NRF24L01µÄSPI¶ÁÊ±Ðò
//****************************************************************************************************
char SPI_Read(char reg)
{
	char reg_val=0;
	RF24L01_CSN_0;           // CSN low, initialize SPI communication...
	SPI_RW(reg);            // Select register to read from..
	reg_val = SPI_RW(0);    // ..then read registervalue
	RF24L01_CSN_1;         // CSN high, terminate SPI communication
	return(reg_val);       // return register value
}
//****************************************************************************************************/
//??:NRF24L01¶ÁÐ´¼Ä´æÆ÷
//****************************************************************************************************/
char SPI_RW_Reg(char reg, char value)
{
	char status1;
	RF24L01_CSN_0;                   // CSN low, init SPI transaction
	status1 = SPI_RW(reg);      // select register
	
	SPI_RW(value);             // ..and write value to it..
	RF24L01_CSN_1;                   // CSN high again
	return(status1);            // return nRF24L01 status uchar
}
//****************************************************************************************************/
//??:uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
//??:ÓÃÓÚ¶ÁÊý¾Ý,reg:¼Ä´æÆ÷µØÖ·,pBuf:´ý¶Á³öÊý¾ÝµÄµØÖ·,uchars:Êý¾Ý³¤¶È
//****************************************************************************************************/
char SPI_Read_Buf(char reg, char *pBuf, char chars)
{
	char status2,uchar_ctr;
	RF24L01_CSN_0;                    		// Set CSN low, init SPI tranaction
	status2 = SPI_RW(reg);       		// Select register to write to and read status uchar
	for(uchar_ctr=0;uchar_ctr<chars;uchar_ctr++)
        {
	pBuf[uchar_ctr] = SPI_RW(0);    // 
        }
	RF24L01_CSN_1;                           
	return(status2);   
}
//*********************************************************************************************************
//??:uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
//??: ÓÃÓÚÐ´Êý¾Ý:Ð´Èë¼Ä´æÆ÷µÄµØÖ·,pBuf:Ð´ÈëÊý¾ÝµÄµØÖ·,uchars:Êý¾Ý³¤¶È
//*********************************************************************************************************/
char SPI_Write_Buf(char reg, char *pBuf, char chars)
{
	char status1,uchar_ctr;
	RF24L01_CSN_0;             //SPI??       
	status1 = SPI_RW(reg);   
	for(uchar_ctr=0; uchar_ctr<chars; uchar_ctr++) //
        {
	SPI_RW(*pBuf++);
        }
	RF24L01_CSN_1;           //??SPI
	return(status1);    		  // 
}
//****************************************************************************************************/
//??:void SetRX_Mode(void)
//??:ÖÃÎ»½ÓÊÕÄ£Ê½ 
//****************************************************************************************************/
void SetRX_Mode(void)
{
	RF24L01_CE_0 ;
       // Delay(1);
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);//
	RF24L01_CE_1; 
	inerDelay_us(130);//??????
}
//******************************************************************************************************/
//??:unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
//??:¶ÁÈ¡Êý¾Ýºó·ÅÈërx_bufÖÐ
//******************************************************************************************************/
char nRF24L01_RxPacket(char* rx_buf)
{
  char revale=0;
	sta=SPI_Read(STATUS);	     // ????????????????		
	if(sta&0x40)                 // ?????????
	{
	    RF24L01_CE_0 ; 			//SPI??
	    SPI_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
	    revale =1;			//????????
	}
        else 
        {
              revale =0;	
        }
	SPI_RW_Reg(WRITE_REG+STATUS,sta);   //??????RX_DR,TX_DS,MAX_PT????1,???1???????
	return revale;
}
//***********************************************************************************************************
//??:void nRF24L01_TxPacket(char * tx_buf)
//??:·¢ËÍ tx_bufÖÐµÄÊý¾Ý
//**********************************************************************************************************/
void nRF24L01_TxPacket(char * tx_buf)
{
	RF24L01_CE_0 ;			//StandBy I??	
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // ???????
	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // ????	
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);   		 // IRQ????????,16?CRC,???
	RF24L01_CE_1;		 //??CE,??????
	 inerDelay_us(10);
}
//****************************************************************************************
//NRF24L01???
//***************************************************************************************/
void init_NRF24L01(void)
{

 	RF24L01_CE_0 ;    // chip enable
 	RF24L01_CSN_1;   // Spi disable 
 	RF24L01_SCK_0;   // Spi clock line init high
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // ?????	
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // ??????
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      //  ??0??	ACK????	
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  //  ??????????0,???????????Page21  
	SPI_RW_Reg(WRITE_REG + RF_CH, 0);        //   ???????2.4GHZ,??????
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //????????,?????32??
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0F);   		//???????2MHZ,????????0dB
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);   		// IRQ????????,16?CRC	,???}
  inerDelay_us(100);
}


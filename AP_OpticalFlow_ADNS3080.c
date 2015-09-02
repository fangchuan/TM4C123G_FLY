/*
 *       AP_OpticalFlow_ADNS3080.cpp - ADNS3080 OpticalFlow Library for
 *       Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 */

#include "Sys.h"
#include "AP_OpticalFlow_ADNS3080.h"
#include "Vector3f.h"
#include "debug.h"
#include "ArduCopter.h"
#include "MPU6050.h"
unsigned char _num_calls;     // used to throttle read down to 20hz

enum Rotation _orientation;

extern float field_of_view;

extern long height_current;
extern _Q_ANGLE  Q_ANGLE;


struct AP_OpticalFlow_Flags {
        uint8_t healthy;    // true if sensor is healthy
    } _flags;



void inerDelay_us(int n)
{
   ROM_SysCtlDelay(80/3*n);
}


//MISO==E0,MOSI==E1,CS==B0,CLK==A6 ,RESET== A7
void SPI_Init()
{
	  RF24L01_IO_set();
    AP_OpticalFlow_ADNS3080_init();
}
void Reset_ADNS3080()
{
	RF24L01_CE_1;
	inerDelay_us(10);
	RF24L01_CE_0;
}
// Public Methods //////////////////////////////////////////////////////////////
// init - initialise sensor
// assumes SPI bus has been initialised but will attempt to initialise 
// nonstandard SPI3 bus if required
void AP_OpticalFlow_ADNS3080_init()
{
    int8_t retry = 0;
    _flags.healthy = false;
	   
	   //Reset_ADNS3080();
	   RF24L01_CSN_1;
	   RF24L01_SCK_0;
	  
	  
    // check 3 times for the sensor on standard SPI bus
     while (!_flags.healthy && retry < 3) {
        if (SPI_Read(ADNS3080_PRODUCT_ID) == 0x17) {
             _flags.healthy = true;
            }
            retry++;
        }


     field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
				
    // configure the sensor
    if (_flags.healthy) {
        // set frame rate to manual
        uint8_t regVal = SPI_Read(ADNS3080_EXTENDED_CONFIG);
        inerDelay_us(50);
        regVal = (regVal & ~0x01) | 0x01;
        SPI_RW_Reg(ADNS3080_EXTENDED_CONFIG, regVal);
        inerDelay_us(50);
       
        // set frame period to 12000 (0x9C40)
			  //the sysytem clock = 80M 
        SPI_RW_Reg(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0x40);
        inerDelay_us(50);
        SPI_RW_Reg(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x9C);
        inerDelay_us(50);

        // set 1600 resolution bit
        regVal = SPI_Read(ADNS3080_CONFIGURATION_BITS);
        inerDelay_us(50);
        regVal |= 0x10;
        SPI_RW_Reg(ADNS3080_CONFIGURATION_BITS, regVal);
        inerDelay_us(50);

        // update scalers
        update_conversion_factors();

        // register the global static read function to be called at 1khz
        //hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_OpticalFlow_ADNS3080::read));
    }else{
        // no connection available.
        //SPI_Init();
         }
				 
     _orientation=ROTATION_NONE;

}


// read latest values from sensor and fill in x,y and totals
void AP_OpticalFlow_ADNS3080_update(void)
{
    Vector3f rot_vector;
	  uint8_t motion_reg;
    int16_t  raw_dx, raw_dy;    // raw sensor change in x and y position (i.e. unrotated)
    surface_quality = SPI_Read(ADNS3080_SQUAL);
	  //debug_printf("surface_quality:%d\n",surface_quality);
    inerDelay_us(50);

    // check for movement, update x,y values
    motion_reg = SPI_Read(ADNS3080_MOTION);	  
	 
    if (((motion_reg & 0x80) != 0) ) {
        raw_dx = ((int8_t)SPI_Read(ADNS3080_DELTA_X));
			  
        inerDelay_us(50);
        raw_dy = ((int8_t)SPI_Read(ADNS3080_DELTA_Y));
			  
    }else{
        raw_dx = 0;
        raw_dy = 0;
    }

    last_update = Get_ms();


		rot_vector=vector3f_init(raw_dx, raw_dy, 0);

    // rotate dx and dy
    rot_vector=Vector3_rotate(_orientation,rot_vector);
    dx = rot_vector.x;
    dy = rot_vector.y;

}

// parent method called at 1khz by periodic process
// this is slowed down to 20hz and each instance's update function is called
// (only one instance is supported at the moment)
void AP_OpticalFlow_ADNS3080_read(void)
{
    _num_calls++;

    if (_num_calls >= AP_OPTICALFLOW_ADNS3080_NUM_CALLS_FOR_20HZ) {
        _num_calls = 0;
        AP_OpticalFlow_ADNS3080_update();
			 
       }
}

// clear_motion - will cause the Delta_X, Delta_Y, and internal motion
// registers to be cleared
void clear_motion()
{
    // writing anything to this register will clear the sensor's motion
    // registers
    SPI_RW_Reg(ADNS3080_MOTION_CLEAR,0xFF); 
    x_cm = 0;
    y_cm = 0;
    dx = 0;
    dy = 0;
}

// get_pixel_data - captures an image from the sensor and stores it to the
// pixe_data array
void print_pixel_data()
{
    int16_t i,j;
    bool isFirstPixel = true;
    uint8_t regValue;
    uint8_t pixelValue;

    // write to frame capture register to force capture of frame
    SPI_RW_Reg(ADNS3080_FRAME_CAPTURE,0x83);

    // wait 3 frame periods + 10 nanoseconds for frame to be captured
    // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.
    // so 500 x 3 + 10 = 1510
    inerDelay_us(1510);

    // display the pixel data
    for (i=0; i<ADNS3080_PIXELS_Y; i++) {
        for (j=0; j<ADNS3080_PIXELS_X; j++) {
            regValue = SPI_Read(ADNS3080_FRAME_CAPTURE);
            if (isFirstPixel && (regValue & 0x40) == 0) {

            }
            isFirstPixel = false;
            pixelValue = ( regValue << 2 );
            //hal.console->print(pixelValue,BASE_DEC);
            if (j!= ADNS3080_PIXELS_X-1)
              
                inerDelay_us(50);
        }
      
    }
}

// updates conversion factors that are dependent upon field_of_view
void update_conversion_factors()
{
    // multiply this number by altitude and pixel change to get horizontal
    // move (in same units as altitude)
    conv_factor = ((1.0f / (float)(ADNS3080_PIXELS_X * AP_OPTICALFLOW_ADNS3080_SCALER_1600))
                   * 2.0f * tanf(field_of_view / 2.0f));
    // 0.00615
    radians_to_pixels = (ADNS3080_PIXELS_X * AP_OPTICALFLOW_ADNS3080_SCALER_1600) / field_of_view;
    // 162.99
}
//==============================================================================
//:uint SPI_RW(uint uchar)
//:NRF24L01的SPI写时序
//******************************************************************************
char SPI_RW(char data)
{	
  char i;
	char temp=0;
	 
  for(i=0;i<8;i++) // output 8-bit
   	{
			RF24L01_SCK_0;
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
	  RF24L01_SCK_1;
   	}
		              // ..then set SCK low again
    return(temp);           		  // return read uchar 
    
}

//****************************************************************************************************
//:uchar SPI_Read(uchar reg)
//:NRF24L01的SPI读时序
//****************************************************************************************************
char SPI_Read(char reg)
{
	char reg_val=0;
	RF24L01_CSN_0;           // CSN low, initialize SPI communication...
	SPI_RW(reg);            // Select register to read from..
	inerDelay_us(50);
	reg_val = SPI_RW(0);    // ..then read registervalue
	RF24L01_CSN_1;         // CSN high, terminate SPI communication
	return(reg_val);       // return register value
}
//****************************************************************************************************/
//:NRF24L01读写寄存器
//****************************************************************************************************/
char SPI_RW_Reg(char reg, char value)
{
	char status1;
	reg=0x80|reg;
	RF24L01_CSN_0;                   // CSN low, init SPI transaction
	status1 = SPI_RW(reg);      // select register
	
	SPI_RW(value);             // ..and write value to it..
	RF24L01_CSN_1;                   // CSN high again
	return(status1);            // return nRF24L01 status uchar
}
//****************************************************************************************************/
//:uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
//:用于读数据,reg:寄存器地址,pBuf:待读出数据的地址,uchars:数据长度
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
//:uint SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
//: 用于写数据:写入寄存器的地址,pBuf:写入数据的地址,uchars:数据长度
//*********************************************************************************************************/
char SPI_Write_Buf(char reg, char *pBuf, char chars)
{
	char status1,uchar_ctr;
	RF24L01_CSN_0;               
	status1 = SPI_RW(reg);   
	for(uchar_ctr=0; uchar_ctr<chars; uchar_ctr++) //
        {
	SPI_RW(*pBuf++);
        }
	RF24L01_CSN_1;           
	return(status1);    		 
}
//===========================RF24L01????==========================================
void RF24L01_IO_set(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);//CS--B0
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);//E1--MOSI A6--CLK  A7--CE		
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7);//E1--MOSI A6--CLK  A7--CE以前是E4E5
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
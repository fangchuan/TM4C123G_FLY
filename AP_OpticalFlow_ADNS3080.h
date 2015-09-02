#ifndef __AP_OPTICALFLOW_ADNS3080_H__
#define __AP_OPTICALFLOW_ADNS3080_H__


#include "AP_OpticalFlow.h"

// timer process runs at 1khz.  50 iterations = 20hz
//#define AP_OPTICALFLOW_ADNS3080_NUM_CALLS_FOR_20HZ          50
#define AP_OPTICALFLOW_ADNS3080_NUM_CALLS_FOR_20HZ          5

// orientations for ADNS3080 sensor
#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD ROTATION_YAW_180
#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD_RIGHT ROTATION_YAW_135
#define AP_OPTICALFLOW_ADNS3080_PINS_RIGHT ROTATION_YAW_90
#define AP_OPTICALFLOW_ADNS3080_PINS_BACK_RIGHT ROTATION_YAW_45
#define AP_OPTICALFLOW_ADNS3080_PINS_BACK ROTATION_NONE
#define AP_OPTICALFLOW_ADNS3080_PINS_BACK_LEFT ROTATION_YAW_315
#define AP_OPTICALFLOW_ADNS3080_PINS_LEFT ROTATION_YAW_270
#define AP_OPTICALFLOW_ADNS3080_PINS_FORWARD_LEFT ROTATION_YAW_225

enum Rotation
{
        ROTATION_NONE=0,	
        ROTATION_YAW_180,
      	ROTATION_YAW_135,
        ROTATION_YAW_90, 
        ROTATION_YAW_45,    
        ROTATION_YAW_315,
        ROTATION_YAW_270,
        ROTATION_YAW_225
};
// field of view of ADNS3080 sensor lenses
#define AP_OPTICALFLOW_ADNS3080_08_FOV 0.202458f        // 11.6 degrees

// scaler - value returned when sensor is moved equivalent of 1 pixel
#define AP_OPTICALFLOW_ADNS3080_SCALER_400   1.1f       // when resolution set to 400
#define AP_OPTICALFLOW_ADNS3080_SCALER_1600  4.4f       // when resolution set to 1600

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X               30
#define ADNS3080_PIXELS_Y               30
#define ADNS3080_CLOCK_SPEED            24000000

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

//==============================================================================
#define  RF24L01_CE_0        IO_Init(0x02)         
#define  RF24L01_CE_1        IO_Init(0x01)         
//=============================RF24L01_CSN??==================================
#define  RF24L01_CSN_0       IO_Init(0x04)          
#define  RF24L01_CSN_1       IO_Init(0x03)    
//=============================RF24L01_SCK======================================
#define  RF24L01_SCK_0       IO_Init(0x06)       
#define  RF24L01_SCK_1       IO_Init(0x05) 
//============================= RF24L01_MOSI??================================
#define  RF24L01_MOSI_0      IO_Init(0x08) 
#define  RF24L01_MOSI_1      IO_Init(0x07) 
//=============================MISO??=========================================//��������
#define  RF24L01_MISO_0      IO_Init(0x10) 
#define  RF24L01_MISO_1      IO_Init(0x09) 
//==========================IRQ??============================================//��������
#define  RF24L01_IRQ_0       IO_Init(0x12)        
#define  RF24L01_IRQ_1       IO_Init(0x11) 

// Extended Configuration bits
#define ADNS3080_SERIALNPU_OFF  0x02


    // constructor
    // AP_OpticalFlow_ADNS3080();

    // initialise the sensor
    void  AP_OpticalFlow_ADNS3080_init();

    // read latest values from sensor and fill in x,y and totals,
    // returns true on successful read
    void  AP_OpticalFlow_ADNS3080_update(void);

    // ADNS3080 specific features

    // called by timer process to read sensor data
    void    AP_OpticalFlow_ADNS3080_read();

    //uint8_t read_register(uint8_t address);
    //void    write_register(uint8_t address, uint8_t value);

    // will cause the x,y, dx, dy, and the sensor's motion registers to
    // be cleared
    void    clear_motion();

    // print_pixel_data - dumps a 30x30 image to the Serial port
    void    print_pixel_data();

    // update conversion factors based on field of view
    void update_conversion_factors();
		
    void Reset_ADNS3080();
    
    void SPI_Init(void);
		
		char SPI_RW(char data);
		
		char SPI_Read(char reg);
		
		char SPI_RW_Reg(char reg, char value);
		
		char SPI_Read_Buf(char reg, char *pBuf, char chars);
		
		char SPI_Write_Buf(char reg, char *pBuf, char chars);
		
		void IO_Init(unsigned char x);
		
		void RF24L01_IO_set(void);
		
		void inerDelay_us(int n);

#endif

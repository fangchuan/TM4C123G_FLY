#include "common.h"
#include "mpu_tool.h"
#include "MPU6050.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "TM4C.h"
#include "UART_user.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ArduCopter.h"
#include "Delay.h"
#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_AHRS_DCM.h"
#include "AP_Math.h"
#include "Parameters.h"
#include "AP_OpticalFlow_ADNS3080.h"
#include "AP_OpticalFlow.h"

long quat[4];
extern _Q_ANGLE Q_ANGLE;
extern _Q_ANGLE  Q_AngleVelocity;
#define OMEGATRANSFORMATION 0.030517578125f
void ouputhandler(void);


/* Send data to the Python client application.
 * Data is formatted as follows:
 * packet[0]    = $
 * packet[1]    = packet type (see packet_type_e)
 * packet[2+]   = data
 */
void send_packet(char packet_type, void *data)
{
#define MAX_BUF_LENGTH  (18)
    char buf[MAX_BUF_LENGTH], length = 0;
    length = length;
    memset(buf, 0, MAX_BUF_LENGTH);
    buf[0] = '$';
    buf[1] = packet_type;

    if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO)
    {
        length = 8;
    }
    else if (packet_type == PACKET_TYPE_QUAT)
    {
        length = 18;
    }
 
	  DMPDriverReport(buf, length, data);
}

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float dmpsafe_asin(float v)
{
	if(isnan(v)){
		return 0.0;
	}
	if (v >= 1.0) {
		return M_PI/2;
	}
	if (v <= -1.0) {
		return -M_PI/2;
	}
	return asin(v);
}

#define q30  1073741824.0f
/**********************************************************
 * for DMP5.1 example
 *********************************************************/
void DMPDriverReport(char* buf, int length, void *data)
{
	//uint8_t i;
	switch(buf[1]){
		case PACKET_TYPE_QUAT:{
			volatile long double q[4];
			//volatile float dmp_roll, dmp_pitch, dmp_yaw;	//unit degree
			/*norm = dmpinvSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
			q[0] = q[0] * norm;
			q[1] = q[1] * norm;
			q[2] = q[2] * norm;
			q[3] = q[3] * norm;*/

		 	 q[0] = ((long*)data)[0]*1.f / q30;
		 	 q[1] = ((long*)data)[1]*1.f / q30;
		 	 q[2] = ((long*)data)[2]*1.f / q30;
		 	 q[3] = ((long*)data)[3]*1.f / q30;

			Q_ANGLE.Y = -(atan2(2.0*(q[0]*q[1] + q[2]*q[3]),
			                       1 - 2.0*(q[1]*q[1] + q[2]*q[2])))* 180/M_PI;
			 // we let safe_asin() handle the singularities near 90/-90 in pitch
			Q_ANGLE.X = -dmpsafe_asin(2.0*(q[0]*q[2] - q[3]*q[1]))* 180/M_PI;

			Q_ANGLE.Z = (atan2(2.0*(q[0]*q[3] + q[1]*q[2]),
			                     1 - 2.0*(q[2]*q[2] + q[3]*q[3])))* 180/M_PI;
		}break;
		case PACKET_TYPE_ACCEL:{

		}break;
		case PACKET_TYPE_GYRO:{

		}break;
	}
}
//陈的四元数算法
void DMPangle(void *data)
{			volatile long double q[4];


		 	 q[0] = ((long*)data)[0]*1.f / q30;
		 	 q[1] = ((long*)data)[1]*1.f / q30;
		 	 q[2] = ((long*)data)[2]*1.f / q30;
		 	 q[3] = ((long*)data)[3]*1.f / q30;

			Q_ANGLE.X = -(atan2(2.0*(q[0]*q[1] + q[2]*q[3]),
			                       1 - 2.0*(q[1]*q[1] + q[2]*q[2])))* 180/M_PI;//原来是Y
			 // we let safe_asin() handle the singularities near 90/-90 in pitch
			Q_ANGLE.Y = -dmpsafe_asin(2.0*(q[0]*q[2] - q[3]*q[1]))* 180/M_PI;//原来是X

			Q_ANGLE.Z = (atan2(2.0*(q[0]*q[3] + q[1]*q[2]),
			                     1 - 2.0*(q[2]*q[2] + q[3]*q[3])))* 180/M_PI;
}
/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
//董的四元数算法
 void print_dmp(long *quat)
 {
     float q[4], norm;
     float roll, pitch, yaw;
     float t13, t21, t22, t23, t33;
     float costheta;
     
     q[0] = (float)quat[0];
     q[1] = (float)quat[1];
     q[2] = (float)quat[2];
     q[3] = (float)quat[3];
     
     norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);	
     q[0] = q[0] / norm;	
     q[1] = q[1] / norm;	
     q[2] = q[2] / norm;	
     q[3] = q[3] / norm;
     
     t13 = 2 * (q[1] * q[3] - q[0] * q[2]);
     t21 = 2 * (q[1] * q[2] - q[0] * q[3]);
     t22 = q[0] * q[0] + q[2] * q[2] - q[1] * q[1] - q[3] * q[3];
     t23 = 2 * (q[2] * q[3] + q[0] * q[1]);
     t33 = q[0] * q[0] + q[3] * q[3] - q[1] * q[1] - q[2] * q[2];
     
     pitch = asin(t23);
     costheta = cos(pitch);
     if(costheta == 0)
     {
         costheta = 1;
     }
     yaw = atan2(-t21 / costheta, t22 / costheta);
     roll = atan2(-t13 / costheta, t33 / costheta);

     pitch *= 180 / M_PI;
     yaw *= 180 / M_PI;
     roll *= 180 / M_PI;
	 Q_ANGLE.Y =  pitch;
     Q_ANGLE.X = roll;
	 Q_ANGLE.Z = yaw;
  
 }
void Self_Test()
{
	while(1)
	{
		if(Get_ms()<25000)
	   {
		if (hal.new_gyro && hal.dmp_on){
    	    	mpu_tool_2();					//update sensor data
    	    	delay_millis(1);
    	    	if((int)(100*Q_ANGLE.X_OFFSET) != (int)(100*Q_ANGLE.X)){
    	    		Q_ANGLE.X_OFFSET = Q_ANGLE.X;}
				if((int)(100*Q_ANGLE.Y_OFFSET) != (int)(100*Q_ANGLE.Y)){
    	    		Q_ANGLE.Y_OFFSET = Q_ANGLE.Y;}
				if((int)(100*Q_ANGLE.Z_OFFSET) != (int)(100*Q_ANGLE.Z)){
    	    		Q_ANGLE.Z_OFFSET =Q_ANGLE.Z;}
				}
           }
		   else 
			   break;
	   }
}
void mpu_tool(void)
{

    unsigned long timestamp;

    unsigned long sensor_timestamp;
    dmp_read_fifo(Q_ANGLE.gyro, Q_ANGLE.accl, Q_ANGLE.quat, &sensor_timestamp, &Q_ANGLE.sensors,
                          &Q_ANGLE.more);
			
			DMPangle(Q_ANGLE.quat);
			Q_AngleVelocity.X=-Q_ANGLE.gyro[0]*OMEGATRANSFORMATION;
			Q_AngleVelocity.Y=-Q_ANGLE.gyro[1]*OMEGATRANSFORMATION;
			Q_AngleVelocity.Z=Q_ANGLE.gyro[2]*OMEGATRANSFORMATION;

      if (!Q_ANGLE.more)
                hal.new_gyro = 0;

      ouputhandler();
		
}

void mpu_tool_2(void)
{

       unsigned long sensor_timestamp;
		   static float pre_ax=0,pre_ay=0,pre_az=0;
       static unsigned long time=0;


        if (hal.new_gyro && hal.dmp_on)
        {

       dmp_read_fifo(Q_ANGLE.gyro, Q_ANGLE.accl, Q_ANGLE.quat, &sensor_timestamp, &Q_ANGLE.sensors,
                          &Q_ANGLE.more);

			DMPangle(Q_ANGLE.quat);

			Q_AngleVelocity.X=-Q_ANGLE.gyro[0]*OMEGATRANSFORMATION;
			Q_AngleVelocity.Y=-Q_ANGLE.gyro[1]*OMEGATRANSFORMATION;
			Q_AngleVelocity.Z=Q_ANGLE.gyro[2]*OMEGATRANSFORMATION;
			
      if (!Q_ANGLE.more)
                hal.new_gyro = 0;

		  ouputhandler();
	}
}
//if success return 1,or 0;
int  dmp_init(void)
{
	    int result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
  
    struct int_param_s int_param;


    int_param.cb = gyro_data_ready_cb;
    int_param.gpio_peripheral = SYSCTL_PERIPH_GPIOF;
    int_param.gpio_port = GPIO_PORTF_BASE;
    int_param.gpio_pin = GPIO_PIN_4;
    int_param.int_num = INT_GPIOF;
    int_param.int_type = GPIO_FALLING_EDGE;
    result = mpu_init(&int_param);
    if (result)
    {
        UARTprintf("mpu init fail!!!!\n");
        return 0;
    }

    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
	  dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
	return 1;
}
void ouputhandler()
{
	  float rate_x,rate_y,rate_z;
	  Vector3f omega;
    rate_x=Q_AngleVelocity.X;
	  rate_y=Q_AngleVelocity.Y;
	  rate_z=Q_AngleVelocity.Z;
	  omega.x=radians(rate_x);omega.y=radians(rate_y);omega.z=radians(rate_z);
	  current_omega=omega;
	  ap_ahrs_dcm.roll_sensor=Q_ANGLE.X*100;
	  ap_ahrs_dcm.pitch_sensor=Q_ANGLE.Y*100;		
    ap_ahrs_dcm.yaw_sensor=Q_ANGLE.Z*100;		
}
void Control()
{

		 if (hal.new_gyro && hal.dmp_on)
        {
          mpu_tool();
					position_update();
		      arducopter_fast_loop();	
            					
		}
}


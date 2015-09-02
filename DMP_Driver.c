#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "DMP_Driver.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "Sys.h"
//#include "../Report/Report.h"

#define DEBUG
#ifdef	DEBUG
#define DEBUG_PRINTLN	UARTprintf
#else
#define DEBUG_PRINTLN(...)	Delay_ms(5)
#endif

void send_packet(char packet_type, void *data)
{
#define MAX_BUF_LENGTH  (18)
    char buf[MAX_BUF_LENGTH];
    uint8_t length;

    memset(buf, 0, MAX_BUF_LENGTH);
    buf[0] = '$';
    buf[1] = packet_type;

    if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO) {
        /*short *sdata = (short*)data;
        buf[2] = (char)(sdata[0] >> 8);
        buf[3] = (char)sdata[0];
        buf[4] = (char)(sdata[1] >> 8);
        buf[5] = (char)sdata[1];
        buf[6] = (char)(sdata[2] >> 8);
        buf[7] = (char)sdata[2];*/
        length = 8;
    } else if (packet_type == PACKET_TYPE_QUAT) {
        /*long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        buf[10] = (char)(ldata[2] >> 24);
        buf[11] = (char)(ldata[2] >> 16);
        buf[12] = (char)(ldata[2] >> 8);
        buf[13] = (char)ldata[2];
        buf[14] = (char)(ldata[3] >> 24);
        buf[15] = (char)(ldata[3] >> 16);
        buf[16] = (char)(ldata[3] >> 8);
        buf[17] = (char)ldata[3];*/
        length = 18;
    }/* else if (packet_type == PACKET_TYPE_TAP) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        length = 4;
    } else if (packet_type == PACKET_TYPE_ANDROID_ORIENT) {
        buf[2] = ((char*)data)[0];
        length = 3;
    } else if (packet_type == PACKET_TYPE_PEDO) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        length = 10;
    } else if (packet_type == PACKET_TYPE_MISC) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        buf[4] = ((char*)data)[2];
        buf[5] = ((char*)data)[3];
        length = 6;
    }*/
    //DMPDriverReport(buf, length, data);
}

/***************************************************************
 *	DMPInitial()
 */
void DMPInitial(){

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     */
    if(dmp_load_motion_driver_firmware()){	//fail
    	while(1);
    }

     /* 2. Push the gyro and accel orientation matrix to the DMP.	*/
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));

     /* 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.*/
    dmp_register_tap_cb(tap_cb);

     /* 4. Call dmp_enable_feature(mask) to enable different features.*/
    dmp_register_android_orient_cb(android_orient_cb);
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);

     /* 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.*/
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);

     /* 6. Call any feature-specific control functions.*/
    mpu_set_dmp_state(1);

     /* To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
}



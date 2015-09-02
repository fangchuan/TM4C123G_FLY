#ifndef __AP_AHRS_DCM__
#define __AP_AHRS_DCM__
#include "Vector3f.h"
#include "Matrix3f.h"
typedef struct _AP_AHRS_DCM
{
	Matrix3f dcm_matrix;
	// Euler angles (radians)
	float roll;
	float pitch;
	float yaw;
	// integer Euler angles (Degrees * 100)
	long roll_sensor;
	long pitch_sensor;
	long yaw_sensor;
	//sample rate N/s (only be used for test)
	int sample_rate;
	
	//accelerometer values in the earth frame in m/s/s
	Vector3f _accel_ef;

} AP_AHRS_DCM;

void _AP_AHRS_DCM_init(AP_AHRS_DCM *ap_ahrs_dcm);
// gyro_vector: radian/s  delta_t: s
void _AP_AHRS_DCM_update(AP_AHRS_DCM *ap_ahrs_dcm, Vector3f *gyro_vector, float delta_t);
void _AP_AHRS_DCM_update_test(AP_AHRS_DCM *ap_ahrs_dcm, Vector3f *gyro_vector, float delta_t);
//void _AP_AHRS_DCM_printf(AP_AHRS_DCM *ap_ahrs_dcm);

#endif

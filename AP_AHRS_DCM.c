#include <stdio.h>

#include "AP_Math.h"
#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_AHRS_DCM.h"

extern void _AP_AHRS_DCM_matrix_update(AP_AHRS_DCM *ap_ahrs_dcm, Vector3f *gyro_vector, float delta_t);
extern void _AP_AHRS_DCM_euler_angles(AP_AHRS_DCM *ap_ahrs_dcm);

void _AP_AHRS_DCM_init(AP_AHRS_DCM *ap_ahrs_dcm)
{
	matrix3f_identity(&(ap_ahrs_dcm->dcm_matrix));//½á¹¹µÄÇ¶Ì×
	_AP_AHRS_DCM_euler_angles(ap_ahrs_dcm);
	ap_ahrs_dcm->sample_rate = 1;
	ap_ahrs_dcm->_accel_ef.x =0; ap_ahrs_dcm->_accel_ef.y = 0;ap_ahrs_dcm->_accel_ef.z = 0;
}

void _AP_AHRS_DCM_update(AP_AHRS_DCM *ap_ahrs_dcm, Vector3f *gyro_vector, float delta_t)
{
	_AP_AHRS_DCM_matrix_update(ap_ahrs_dcm, gyro_vector, delta_t);
	_AP_AHRS_DCM_euler_angles(ap_ahrs_dcm);
}

void _AP_AHRS_DCM_update_test(AP_AHRS_DCM *ap_ahrs_dcm, Vector3f *gyro_vector, float delta_t)
{
	long num = (int)(ap_ahrs_dcm->sample_rate * delta_t);
	long i;

	for(i = 0; i < num; i++)
	{
		_AP_AHRS_DCM_update(ap_ahrs_dcm, gyro_vector, (float)1 / ap_ahrs_dcm->sample_rate);
	}
}


void _AP_AHRS_DCM_matrix_update(AP_AHRS_DCM *ap_ahrs_dcm, Vector3f *gyro_vector, float delta_t)
{
	Vector3f rotate_angle = vector3f_multiply_by_coefficient(gyro_vector, delta_t);
	matrix3f_rotate(&ap_ahrs_dcm->dcm_matrix, &rotate_angle);
}

void _AP_AHRS_DCM_euler_angles(AP_AHRS_DCM *ap_ahrs_dcm)
{
	matrix3f_to_euler(&ap_ahrs_dcm->dcm_matrix, &ap_ahrs_dcm->roll, &ap_ahrs_dcm->pitch, &ap_ahrs_dcm->yaw);

	ap_ahrs_dcm->roll_sensor     = (long)(degrees(ap_ahrs_dcm->roll)  * 100);
	ap_ahrs_dcm->pitch_sensor    = (long)(degrees(ap_ahrs_dcm->pitch) * 100);
	ap_ahrs_dcm->yaw_sensor      = (long)(degrees(ap_ahrs_dcm->yaw)   * 100);

	if (ap_ahrs_dcm->yaw_sensor < 0)
		ap_ahrs_dcm->yaw_sensor += 36000;
}
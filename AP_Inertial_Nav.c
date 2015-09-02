#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_AHRS_DCM.h"
#include "AP_Baro.h"
#include "AP_BufferFloat.h"
#include "confige.h"
#include "AP_Math.h"
#include <string.h>
#include <stdio.h>

#include "AP_Inertial_Nav.h"
void ap_inertial_nav_check_baro(AP_Inertial_Nav *ap_inertial_nav);
void ap_inertial_nav_correct_with_baro(AP_Inertial_Nav *ap_inertial_nav, float baro_alt, float dt);
void ap_inertial_nav_set_altitude(AP_Inertial_Nav *ap_inertial_nav, float new_altitude);

//
// Private methods
//

// update_gains - update gains from time constant (given in seconds)
void ap_inertial_nav_update_gains(AP_Inertial_Nav *ap_inertial_nav);
// set_velocity_z - get latest climb rate (in cm/s)
void ap_inertial_nav_set_velocity_z(AP_Inertial_Nav *ap_inertial_nav, float z );

// init - initialise library
void ap_inertial_nav_init(AP_Inertial_Nav *ap_inertial_nav, AP_AHRS_DCM *ap_ahrs_dcm, AP_Baro *ap_baro)
{
	ap_inertial_nav->accel_correction.x = ap_inertial_nav->accel_correction.y 
					= ap_inertial_nav->accel_correction.z = 0.0F;

	ap_inertial_nav->_ahrs = ap_ahrs_dcm;
	ap_inertial_nav->_baro = ap_baro;

	ap_inertial_nav->_time_constant_z = AP_INTERTIALNAV_TC_Z;
	ap_inertial_nav_update_gains(ap_inertial_nav);
	ap_inertial_nav->_baro_last_update = 0;
	ap_bufferfloat_init(&ap_inertial_nav->_hist_position_estimate_z, 15);
	ap_inertial_nav->_first_read_baro = 0;

	ap_inertial_nav->_position_base.x = ap_inertial_nav->_position_base.y
			= ap_inertial_nav->_position_base.z = 0;
	ap_inertial_nav->_position_correction.x = ap_inertial_nav->_position_correction.y
			= ap_inertial_nav->_position_correction.z = 0;
	ap_inertial_nav->_velocity.x = ap_inertial_nav->_velocity.y 
			= ap_inertial_nav->_velocity.z = 0;
}

// update - updates velocities and positions using latest info from ahrs, ins and barometer if new data is available;
void ap_inertial_nav_update(AP_Inertial_Nav *ap_inertial_nav, float dt)
{
	Vector3f acc_corr = ap_inertial_nav->accel_correction;
	Vector3f accel_ef;
	Vector3f velocity_increase;
	Matrix3f dcm;
	Vector3f vector3f_tmp;

	// discard samples where dt is too large

	if( dt > 0.1 ) {
		return;
	}

	// check barometer
	ap_inertial_nav_check_baro(ap_inertial_nav);

	// convert accelerometer readings to earth frame
	dcm = ap_inertial_nav->_ahrs->dcm_matrix;
	accel_ef = ap_inertial_nav->_ahrs->_accel_ef;

	// remove influence of gravity
	accel_ef.z += GRAVITY;
	accel_ef = vector3f_multiply_by_coefficient(&accel_ef, 100.0F);

	// get earth frame accelerometer correction
	ap_inertial_nav->accel_correction_ef = matrix3f_mul_transpose(&dcm, &acc_corr);

	// calculate velocity increase adding new acceleration from accelerometers
	//velocity_increase = (-accel_ef + accel_correction_ef) * dt;
	vector3f_tmp = vector3f_multiply_by_coefficient(&accel_ef, -1);
	vector3f_tmp = vector3f_plus(&vector3f_tmp, &ap_inertial_nav->accel_correction_ef);
	velocity_increase = vector3f_multiply_by_coefficient(&vector3f_tmp, dt);

	// calculate new estimate of position
	//_position_base += (_velocity + velocity_increase*0.5) * dt;
	vector3f_tmp = vector3f_multiply_by_coefficient(&velocity_increase, 0.5);
	vector3f_tmp = vector3f_plus(&ap_inertial_nav->_velocity, &vector3f_tmp);
	vector3f_tmp = vector3f_multiply_by_coefficient(&vector3f_tmp, dt);
	ap_inertial_nav->_position_base = vector3f_plus(&ap_inertial_nav->_position_base, &vector3f_tmp);

	// calculate new velocity
	//_velocity += velocity_increase;
	ap_inertial_nav->_velocity = vector3f_plus(&ap_inertial_nav->_velocity, &velocity_increase);

	// store 3rd order estimate (i.e. estimated vertical position) for future use
	//_hist_position_estimate_z.add(_position_base.z);
	ap_bufferfloat_add(&ap_inertial_nav->_hist_position_estimate_z, ap_inertial_nav->_position_base.z);
}

//
// Z Axis methods
//

// check_baro - check if new baro readings have arrived and use them to correct vertical accelerometer offsets
void ap_inertial_nav_check_baro(AP_Inertial_Nav *ap_inertial_nav)
{
	unsigned long baro_update_time;

	if( ap_inertial_nav->_baro == NULL )
		return;

	// calculate time since last baro reading
	baro_update_time = ap_inertial_nav->_baro->last_update;
	if( baro_update_time != ap_inertial_nav->_baro_last_update ) {
		float dt = (float)(baro_update_time - ap_inertial_nav->_baro_last_update) / 1000.0F;
		// call correction method
		ap_inertial_nav_correct_with_baro(ap_inertial_nav, ap_inertial_nav->_baro->altitude, dt);
		ap_inertial_nav->_baro_last_update = baro_update_time;
	}
}


// correct_with_baro - modifies accelerometer offsets using barometer.  dt is time since last baro reading
void ap_inertial_nav_correct_with_baro(AP_Inertial_Nav *ap_inertial_nav, float baro_alt, float dt)
{
	float hist_position_base_z;
	float accel_ef_z_correction;
	Matrix3f dcm;
	Vector3f accel_corr;
	float err;

	// discard samples where dt is too large

	if( dt > 0.5 ) {
		return;
	}

	// discard first 10 reads but perform some initialisation
	if( ap_inertial_nav->_first_read_baro <= 10 ) {
		ap_inertial_nav_set_altitude(ap_inertial_nav, baro_alt);
		ap_inertial_nav->_first_read_baro++;
	}

	// get dcm matrix
	dcm = ap_inertial_nav->_ahrs->dcm_matrix;

	// 3rd order samples (i.e. position from baro) are delayed by 150ms (15 iterations at 100hz)
	// so we should calculate error using historical estimates
	if( ap_inertial_nav->_hist_position_estimate_z.num_items >= 15 ) {
		hist_position_base_z = ap_bufferfloat_peek(&ap_inertial_nav->_hist_position_estimate_z, 14);
	}else{
		hist_position_base_z = ap_inertial_nav->_position_base.z;
	}

	// calculate error in position from baro with our estimate
	err = baro_alt - (hist_position_base_z + ap_inertial_nav->_position_correction.z);

	// retrieve the existing accelerometer corrections
	accel_corr = ap_inertial_nav->accel_correction;

	// calculate the accelerometer correction from this iteration in the earth frame
	accel_ef_z_correction = err * ap_inertial_nav->_k3_z * dt;

	// rotate the correction into the body frame (note: this is a shortened form of dcm.mul_transpose(..) because we have only one axis
	accel_corr.x += accel_ef_z_correction * dcm.c.x;
	accel_corr.y += accel_ef_z_correction * dcm.c.y;
	accel_corr.z += accel_ef_z_correction * dcm.c.z;

	// ensure corrections are reasonable
	accel_corr.x = constrain(accel_corr.x,-AP_INTERTIALNAV_ACCEL_CORR_MAX,AP_INTERTIALNAV_ACCEL_CORR_MAX);
	accel_corr.y = constrain(accel_corr.y,-AP_INTERTIALNAV_ACCEL_CORR_MAX,AP_INTERTIALNAV_ACCEL_CORR_MAX);
	accel_corr.z = constrain(accel_corr.z,-AP_INTERTIALNAV_ACCEL_CORR_MAX,AP_INTERTIALNAV_ACCEL_CORR_MAX);

	// set the parameter to include the corrections
	ap_inertial_nav->accel_correction = accel_corr;

	// correct velocity
	ap_inertial_nav->_velocity.z += (err * ap_inertial_nav->_k2_z) * dt;

	// correct position
	ap_inertial_nav->_position_correction.z += err * ap_inertial_nav->_k1_z * dt;
}

// set_altitude - set base altitude estimate in cm
void ap_inertial_nav_set_altitude(AP_Inertial_Nav *ap_inertial_nav, float new_altitude)
{
	ap_inertial_nav->_position_base.z = new_altitude;
	ap_inertial_nav->_position_correction.z = 0;
}

//
// Private methods
//

// update_gains - update gains from time constant (given in seconds)
void ap_inertial_nav_update_gains(AP_Inertial_Nav *ap_inertial_nav)
{
	// Z axis time constant
	if( ap_inertial_nav->_time_constant_z == 0 ) {
		ap_inertial_nav->_k1_z = ap_inertial_nav->_k2_z = ap_inertial_nav->_k3_z = 0;
	}else{
		ap_inertial_nav->_k1_z = 3 / ap_inertial_nav->_time_constant_z;
		ap_inertial_nav->_k2_z = 3 / (ap_inertial_nav->_time_constant_z * ap_inertial_nav->_time_constant_z);
		ap_inertial_nav->_k3_z = 1 / (ap_inertial_nav->_time_constant_z * ap_inertial_nav->_time_constant_z 
								* ap_inertial_nav->_time_constant_z);
	}
}

// set_velocity_z - get latest climb rate (in cm/s)
void ap_inertial_nav_set_velocity_z(AP_Inertial_Nav *ap_inertial_nav, float z )
{
	ap_inertial_nav->_velocity.z = z;
}


// void ap_inertial_nav_print(AP_Inertial_Nav *ap_inertial_nav)
// {
// 	Vector3f *pos_base = &ap_inertial_nav->_position_base;
// 	Vector3f *pos_corr = &ap_inertial_nav->_position_correction;
// 	Vector3f *v = &ap_inertial_nav->_velocity;
// 	printf("=================AP_INERTIAL_NAV===============\n");
// 	printf("position_base:       [%f, %f, %f]\n", pos_base->x, pos_base->y, pos_base->z);
// 	printf("position_correction: [%f, %f, %f]\n", pos_corr->x, pos_corr->y, pos_corr->z);
// 	printf("velocity:            [%f, %f, %f]\n", v->x, v->y, v->z);
// 	printf("================================================\n");
// }

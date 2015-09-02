#ifndef __AP_INERTIALNAV__
#define __AP_INERTIALNAV__
#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_AHRS_DCM.h"
#include "AP_Baro.h"
#include "AP_BufferFloat.h"
typedef struct _AP_Inertial_Nav
{
	// public variables
	Vector3f				accel_correction;          // acceleration corrections
	Vector3f                accel_correction_ef;        // earth frame accelerometer corrections. here for logging purposes only

	//protected:
	AP_AHRS_DCM*             _ahrs;                      // pointer to ahrs object
	//AP_InertialSensor*      _ins;                       // pointer to inertial sensor
	AP_Baro*                _baro;                      // pointer to barometer

	// Z Axis specific variables
	float					_time_constant_z;           // time constant for vertical corrections
	float                   _k1_z;                      // gain for vertical position correction
	float                   _k2_z;                      // gain for vertical velocity correction
	float                   _k3_z;                      // gain for vertical accelerometer offset correction
	unsigned long           _baro_last_update;           // time of last barometer update
	AP_BufferFloat		    _hist_position_estimate_z;  // buffer of historic accel based altitudes to account for lag
	unsigned char			_first_read_baro;

	// general variables
	Vector3f                _position_base;             // position estimate
	Vector3f                _position_correction;       // sum of correction to _comp_h from delayed 1st order samples    
	Vector3f                _velocity;                  // latest velocity estimate (integrated from accelerometer values)
} AP_Inertial_Nav;

#define AP_INTERTIALNAV_TC_Z    1.0F // default time constant for complementary filter's Z axis
#define AP_INTERTIALNAV_ACCEL_CORR_MAX 300.0F    // max allowed accelerometer offset correction

// init - initialise library
void ap_inertial_nav_init(AP_Inertial_Nav *ap_inertial_nav, AP_AHRS_DCM *ap_ahrs_dcm, AP_Baro *ap_baro);
// update - updates velocities and positions using latest info from ahrs, ins and barometer if new data is available;
void ap_inertial_nav_update(AP_Inertial_Nav *ap_inertial_nav, float dt);
//printf information to stdout

//void ap_inertial_nav_print(AP_Inertial_Nav *ap_inertial_nav);


#endif

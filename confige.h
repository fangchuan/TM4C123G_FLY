#ifndef __CONFIG__
#define  __CONFIG__

// Good for smaller payload motors.

# define STABILIZE_ROLL_P         2.0//2.5
# define STABILIZE_ROLL_I          0.0
# define STABILIZE_ROLL_IMAX    	8.0            // degrees

# define STABILIZE_PITCH_P         2.0//2.5
# define STABILIZE_PITCH_I         0.0
# define STABILIZE_PITCH_IMAX   	8.0            // degrees

# define STABILIZE_YAW_P           2.0//2.5            // increase for more aggressive Yaw Hold, decrease if it's bouncy
# define STABILIZE_YAW_I           0.0
# define STABILIZE_YAW_IMAX        8.0            // degrees * 100

# define ACRO_P                 4.5


#define RATE_ROLL_P        		0.09f//
#define RATE_ROLL_I        		0.006F//
#define RATE_ROLL_D        	  0.0012f//
#define RATE_ROLL_IMAX        50.0F//                    // degrees

#define RATE_PITCH_P       		0.09f//
#define RATE_PITCH_I       		0.006f//
#define RATE_PITCH_D       		0.0012f//  
#define RATE_PITCH_IMAX       50.0F//             // degrees

#define RATE_YAW_P              0.1//0.1
#define RATE_YAW_I              0.006F//
#define RATE_YAW_D              0.0012F//0.001//0.0045
#define RATE_YAW_IMAX           50.0F //         			// degrees

#define HEIGHT_P				0.1f//
#define HEIGHT_I				0.05f	//0.01
#define HEIGHT_D				0.08f//
#define HEIGHT_I_MAX			60.0F
//#define HEIGHT_BASE				790//790
 
#define TARGET_HEIGHT   500

#define POINT_X_P         0.5f//0.5
#define POINT_X_I         0//.06f
#define POINT_X_D         0.2f//
#define POINT_X_I_MAX     50.0F

#define POINT_Y_P         0.5f//0.5
#define POINT_Y_I         0//.06f
#define POINT_Y_D         0.2f//
#define POINT_Y_I_MAX     50.0F

//angle and range
#define MAX_INPUT_ROLL_ANGLE	4500
#define MAX_INPUT_PITCH_ANGLE	4500
#define MAX_INPUT_YAW_ANGLE		4500

#define MINIMUM_THROTTLE       130
#define MAXIMUM_THROTTLE       1000

#define GRAVITY 9.80665F

#endif

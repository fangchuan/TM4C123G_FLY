#include "AP_OpticalFlow.h"
#include "debug.h"
#define FORTYFIVE_DEGREES 0.78539816f

float conv_factor;              // multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
float radians_to_pixels;
float _last_roll;
float _last_pitch;
float _last_altitude;
uint8_t  surface_quality;   // image quality (below 15 you can't trust the dx,dy values returned)
int16_t  dx,dy;             // rotated change in x and y position
int16_t  x,y;
uint32_t last_update;       // millis() time of last update
float    field_of_view;     // field of view in Radians
float vlon=0,vlat=0;
// public variables for reporting purposes
float    x_cm, y_cm;    // x,y position in cm
extern long current_point_x,current_point_y;

// sets field of view of sensor
void    set_field_of_view(const float fov) 
{       
        field_of_view = fov; 
};
// updates internal lon and lat with estimation based on optical flow
void ADNS3080_update_position(float roll, float pitch,float altitude)
{
	  float diff_roll     = roll  - _last_roll;
    float diff_pitch    = pitch - _last_pitch;
    float change_x, change_y;   // actual change in x, y coordinates
    float avg_altitude=0;  
    static float vlon1=0,vlat1=0;
	 
    // only update position if surface quality is good and angle is not
    // over 45 degrees
    if( surface_quality >= 10 && fabsf(roll) <= FORTYFIVE_DEGREES
     && fabsf(pitch) <= FORTYFIVE_DEGREES ) {
	    altitude = max(altitude,0);
      avg_altitude = (altitude + _last_altitude)*0.5f;
			 //debug_printf("avg_altitude:%f\n",avg_altitude);
        // change in position is actual change measured by sensor (i.e. dx, dy) minus expected change due to change in roll, pitch
        change_x = dx - (diff_roll * radians_to_pixels);
			  //debug_printf("dx:%d\n",dx);
			  //debug_printf("change_x:%f\n",change_x);
        change_y = dy - (diff_pitch * radians_to_pixels);
			  //debug_printf("dy:%d\n",dy);
        //debug_printf("change_y:%f\n",change_y);

        // convert raw change to horizontal movement in cm
        // perhaps this altitude should actually be the distance to the
        // ground?  i.e. if we are very rolled over it should be longer
        x_cm =  -change_x * avg_altitude * conv_factor;
        // for example if you are leaned over at 45 deg the ground will
        // appear farther away and motion from opt flow sensor will be less
        y_cm =  -change_y * avg_altitude * conv_factor;

			  vlon1 += x_cm;
			  vlat1 += y_cm;
				if(fabsf(vlat1 - vlat) >= 10 && fabsf(vlon1 - vlon) >= 10){    
                                     }
				else  {   
				 vlon = vlon1;
	       vlat = vlat1;
				}

    }
    current_point_x = vlon;
	  current_point_y = vlat;
    _last_altitude = altitude;
    _last_roll = roll;
    _last_pitch = pitch;

    //debug_printf("vlon:%f\n",vlon);
		//debug_printf("vlat:%f\n",vlat);
// 		debug_printf("current_point_x:%f\n",vlon);
// 		debug_printf("current_point_y:%f\n",vlat);

}
void position_update()
{
	  AP_OpticalFlow_ADNS3080_read();
	  
}
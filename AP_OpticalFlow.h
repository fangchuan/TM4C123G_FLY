#ifndef __AP_OPTICALFLOW_H__
#define __AP_OPTICALFLOW_H__
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_OpticalFlow.cpp - OpticalFlow Base Class for Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       Methods:
 *               init() : initializate sensor and library.
 *               read   : reads latest value from OpticalFlow and
 *                        stores values in x,y, surface_quality parameter
 *               read_register()  : reads a value from the sensor (will be
 *                                  sensor specific)
 *               write_register() : writes a value to one of the sensor's
 *                                  register (will be sensor specific)
 */

#include "AP_Math.h"
#include "math.h"
#include "AP_OpticalFlow_ADNS3080.h"
#include "stdint.h"
#include "stdbool.h"




void    set_field_of_view(const float fov) ;

    // read latest values from sensor and fill in x,y and totals.

    // updates internal lon and lat with estimation based on optical flow
void    ADNS3080_update_position(float roll, float pitch,float altitude);
void    position_update(void);
    // public variables
extern     uint8_t  surface_quality;   // image quality (below 15 you can't trust the dx,dy values returned)
extern     int16_t  dx,dy;             // rotated change in x and y position
extern     uint32_t last_update;       // millis() time of last update
extern     float    field_of_view;     // field of view in Radians
extern     float    x_cm, y_cm; 
extern     int16_t    x,y;

extern float conv_factor;              // multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
extern float radians_to_pixels;
extern float _last_roll;
extern float _last_pitch;
extern float _last_altitude;

#endif

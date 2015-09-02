#include "mathext.h"

float degree2radian(float degree)
{
    return degree / 180 * M_PI;
}

float radian2degree(float radian)
{
    return radian / M_PI * 180;
}

float degree_0_360(float degree)
{
    while(degree < 0)
    {
        degree += 360;
    }
    while(degree > 360)
    {
        degree -= 360;
    }
}

float degree_n180_180(float degree)
{
    while(degree < -180)
    {
        degree += 360;
    }
    while(degree > 180)
    {
        degree -= 360;
    }
}
#ifndef __AP_MATH__
#define __AP_MATH__

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define RADX100 0.000174532925//rad to degrees/100
#define DEGX100 5729.57795//degrees to rad*100
#define HALF_SQRT_2 0.70710678118654757
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)

float safe_asin(float v);
float safe_sqrt(float v);
long wrap_360(long error);
long wrap_180(long error);
char long_to_string(unsigned char *s,long x);
//int isnan(float f);
int finite(float f);
int float_to_str(float f, int precision, char *str, int size);

#endif

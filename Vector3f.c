#include "Vector3f.h"
#include "AP_Math.h"
#include <math.h>

int vector3f_equal(Vector3f *v1, Vector3f *v2)
{
	return v1->x == v2->x && v1->y == v2->y && v1->z == v2->z;
}
Vector3f vector3f_init(int v1,int v2,int v3)
{
	Vector3f v;
	v.x=v1;
	v.y=v2;
	v.z=v3;
	return v;
}

Vector3f vector3f_plus(Vector3f *v1, Vector3f *v2)
{
	Vector3f v;
	v.x = v1->x + v2->x;
	v.y = v1->y + v2->y;
	v.z = v1->z + v2->z;
	return v;
}
Vector3f vector3f_minus(Vector3f *v1, Vector3f *v2)
{
	Vector3f v;
	v.x = v1->x - v2->x;
	v.y = v1->y - v2->y;
	v.z = v1->z - v2->z;
	return v;
}
float vector3f_multiply(Vector3f *v1, Vector3f *v2)
{
	return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}
Vector3f vector3f_multiply_by_coefficient(Vector3f *v, float co)
{
	Vector3f v1;
	v1.x = v->x * co;
	v1.y = v->y * co;
	v1.z = v->z * co;
	return v1;
}
Vector3f vector3f_divide_by_coefficient(Vector3f *v, float co)
{
	Vector3f v1;
	v1.x = v->x / co;
	v1.y = v->y / co;
	v1.z = v->z / co;
	return v1;
}
Vector3f vector3f_cross_product(Vector3f *v1, Vector3f *v2)
{
	//Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
	Vector3f v;
	v.x = v1->y * v2->z - v1->z * v2->y;
	v.y = v1->z * v2->x - v1->x * v2->z;
	v.z = v1->x * v2->y - v1->y * v2->x;
	return v;
}
float vector3f_length(Vector3f *v)
{
	return (float)sqrt(vector3f_multiply(v, v));
}

int vector3f_is_nan(Vector3f *v)
{
	return isnan(v->x) || isnan(v->y) || isnan(v->z);
}
int vector3f_is_inf(Vector3f *v)
{
	return finite(v->x) || finite(v->y) || finite(v->z);
}

Vector3f Vector3_rotate(enum Rotation rotation,Vector3f v)
{
    float tmp;
    switch (rotation) {
    case ROTATION_NONE:
        return v;
    case ROTATION_YAW_45: {
        tmp = HALF_SQRT_2*(v.x - v.y);
        v.y   = HALF_SQRT_2*(v.x + v.y);
        v.x = tmp;
        return v;
    }
    case ROTATION_YAW_90: {
        tmp = v.x; v.x = -v.y; v.y = tmp;
        return v;
    }
    case ROTATION_YAW_135: {
        tmp = -HALF_SQRT_2*(v.x + v.y);
        v.y   =  HALF_SQRT_2*(v.x - v.y);
        v.x = tmp;
        return v;
    }
    case ROTATION_YAW_180:
        v.x = -v.x; v.y = -v.y;
        return v;
    case ROTATION_YAW_225: {
        tmp = HALF_SQRT_2*(v.y - v.x);
        v.y   = -HALF_SQRT_2*(v.x + v.y);
        v.x = tmp;
        return v;
    }
    case ROTATION_YAW_270: {
        tmp = v.x; v.x = v.y; v.y = -tmp;
        return v;
    }
    case ROTATION_YAW_315: {
        tmp = HALF_SQRT_2*(v.x + v.y);
        v.y   = HALF_SQRT_2*(v.y - v.x);
        v.x = tmp;
        return v;
    }

    }
}
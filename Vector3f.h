#ifndef __VECTOR3F__
#define __VECTOR3F__
#include "AP_OpticalFlow_ADNS3080.h"
typedef struct _Vector3f
{
	float x,y,z;
}Vector3f;

int vector3f_equal(Vector3f *v1, Vector3f *v2);
Vector3f vector3f_plus(Vector3f *v1, Vector3f *v2);
Vector3f vector3f_minus(Vector3f *v1, Vector3f *v2);
float vector3f_multiply(Vector3f *v1, Vector3f *v2);
Vector3f vector3f_multiply_by_coefficient(Vector3f *v, float co);
Vector3f vector3f_divide_by_coefficient(Vector3f *v, float co);
Vector3f vector3f_cross_product(Vector3f *v1, Vector3f *v2);
float vector3f_length(Vector3f *v);
Vector3f vector3f_init(int v1,int v2,int v3);
int vector3f_is_nan(Vector3f *v);
int vector3f_is_inf(Vector3f *v);
Vector3f Vector3_rotate(enum Rotation rotation,Vector3f v);

#endif

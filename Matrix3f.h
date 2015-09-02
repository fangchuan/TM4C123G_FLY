#ifndef __MATRIX3F__
#define __MATRIX3F__
#include "Vector3f.h"
typedef struct _Matrix3f
{
	Vector3f a, b, c;
} Matrix3f;

int matrix3f_equal(Matrix3f *m1, Matrix3f *m2);
int matrix3f_is_nan(Matrix3f *m);

Matrix3f matrix3f_plus(Matrix3f *m1, Matrix3f *m2);
Matrix3f matrix3f_minus(Matrix3f *m1, Matrix3f *m2);
Matrix3f matrix3f_multiply_by_coefficient(Matrix3f *m, float co);
Matrix3f matrix3f_divide_by_coefficient(Matrix3f *m, float co);

Matrix3f matrix3f_multiply(Matrix3f *m1, Matrix3f *m2);
Vector3f matrix3f_multiply_by_vector3f(Matrix3f *m, Vector3f *v);
Vector3f matrix3f_mul_transpose(Matrix3f *m, Vector3f *v);
Matrix3f matrix3f_transposed(Matrix3f *m);

void matrix3f_zero(Matrix3f *m);
void matrix3f_identity(Matrix3f *m);

void matrix3f_rotate(Matrix3f *m, Vector3f *v);
void matrix3f_from_euler(Matrix3f *m, float roll, float pitch, float yaw);
void matrix3f_to_euler(Matrix3f *m, float *roll, float *pitch, float *yaw);

#endif

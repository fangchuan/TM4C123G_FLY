#include "Vector3f.h"
#include "Matrix3f.h"
#include "AP_Math.h"
#include <math.h>
#include <string.h>

int matrix3f_equal(Matrix3f *m1, Matrix3f *m2)
{
	return vector3f_equal(&m1->a, &m2->a) || vector3f_equal(&m1->b, &m2->b) || vector3f_equal(&m1->c, &m2->c);
}

int matrix3f_is_nan(Matrix3f *m)
{
	return vector3f_is_nan(&m->a) || vector3f_is_nan(&m->b) || vector3f_is_nan(&m->c);
}


Matrix3f matrix3f_plus(Matrix3f *m1, Matrix3f *m2)
{
	Matrix3f m;
	m.a = vector3f_plus(&m1->a, &m2->a);
	m.b = vector3f_plus(&m1->b, &m2->b);
	m.c = vector3f_plus(&m1->c, &m2->c);
	return m;
}

Matrix3f matrix3f_minus(Matrix3f *m1, Matrix3f *m2)
{
	Matrix3f m;
	m.a = vector3f_minus(&m1->a, &m2->a);
	m.b = vector3f_minus(&m1->b, &m2->b);
	m.c = vector3f_minus(&m1->c, &m2->c);
	return m;
}

Matrix3f matrix3f_multiply_by_coefficient(Matrix3f *m, float co)
{
	Matrix3f m1;
	m1.a = vector3f_multiply_by_coefficient(&m->a, co);
	m1.b = vector3f_multiply_by_coefficient(&m->b, co);
	m1.c = vector3f_multiply_by_coefficient(&m->c, co);
	return m1;
}

Matrix3f matrix3f_divide_by_coefficient(Matrix3f *m, float co)
{
	Matrix3f m1;
	m1.a = vector3f_divide_by_coefficient(&m->a, co);
	m1.b = vector3f_divide_by_coefficient(&m->b, co);
	m1.c = vector3f_divide_by_coefficient(&m->c, co);
	return m1;
}


Matrix3f matrix3f_multiply(Matrix3f *m1, Matrix3f *m2)
{
	Matrix3f m;

    m.a.x = m1->a.x * m2->a.x + m1->a.y * m2->b.x + m1->a.z * m2->c.x;
	m.a.y = m1->a.x * m2->a.y + m1->a.y * m2->b.y + m1->a.z * m2->c.y;
	m.a.z = m1->a.x * m2->a.z + m1->a.y * m2->b.z + m1->a.z * m2->c.z;

	m.b.x = m1->b.x * m2->a.x + m1->b.y * m2->b.x + m1->b.z * m2->c.x;
	m.b.y = m1->b.x * m2->a.y + m1->b.y * m2->b.y + m1->b.z * m2->c.y;
	m.b.z = m1->b.x * m2->a.z + m1->b.y * m2->b.z + m1->b.z * m2->c.z;

	m.c.x = m1->c.x * m2->a.x + m1->c.y * m2->b.x + m1->c.z * m2->c.x;
	m.c.y = m1->c.x * m2->a.y + m1->c.y * m2->b.y + m1->c.z * m2->c.y;
	m.c.z = m1->c.x * m2->a.z + m1->c.y * m2->b.z + m1->c.z * m2->c.z;
	return m;
}

Vector3f matrix3f_multiply_by_vector3f(Matrix3f *m, Vector3f *v)
{
	Vector3f v1;
	v1.x = m->a.x * v->x + m->a.y * v->y + m->a.z * v->z;
	v1.y = m->b.x * v->x + m->b.y * v->y + m->b.z * v->z;
	v1.z = m->c.x * v->x + m->c.y * v->y + m->c.z * v->z;
	return v1;
}

Vector3f matrix3f_mul_transpose(Matrix3f *m, Vector3f *v)
{
	Vector3f v1;
	v1.x = m->a.x * v->x + m->b.x * v->y + m->c.x * v->z;
	v1.y = m->a.y * v->x + m->b.y * v->y + m->c.y * v->z;
	v1.z = m->a.z * v->x + m->b.z * v->y + m->c.z * v->z;
	return v1;
}

Matrix3f matrix3f_transposed(Matrix3f *m)
{
	Matrix3f m1;
	/*= 
	{
		{m->a.x, m->b.x, m->c.x},
		{m->a.y, m->b.y, m->c.y},
		{m->a.z, m->b.z, m->c.z},
	};*/
	
	m1.a.x = m->a.x;
	m1.a.y = m->a.y;
	m1.a.z = m->a.z;
	
	m1.b.x = m->b.x;
	m1.b.y = m->b.y;
	m1.b.z = m->b.z;
	
	m1.c.x = m->c.x;
	m1.c.y = m->c.y;
	m1.c.z = m->c.z;

	return m1;
}


void matrix3f_zero(Matrix3f *m)
{
	m->a.x = 0;m->a.y =0; m->a.z = 0;
	m->b.x =0; m->b.y = 0;m->b.z = 0;
	m->c.x = 0;m->c.y =0; m->c.z = 0;
}

void matrix3f_identity(Matrix3f *m)
{
	matrix3f_zero(m);
	m->a.x =1; m->b.y =1; m->c.z = 1;
}


void matrix3f_rotate(Matrix3f *m, Vector3f *v)
{
	Matrix3f temp_matrix;
	temp_matrix.a.x = m->a.y * v->z - m->a.z * v->y;
	temp_matrix.a.y = m->a.z * v->x - m->a.x * v->z;
	temp_matrix.a.z = m->a.x * v->y - m->a.y * v->x;
	temp_matrix.b.x = m->b.y * v->z - m->b.z * v->y;
	temp_matrix.b.y = m->b.z * v->x - m->b.x * v->z;
	temp_matrix.b.z = m->b.x * v->y - m->b.y * v->x;
	temp_matrix.c.x = m->c.y * v->z - m->c.z * v->y;
	temp_matrix.c.y = m->c.z * v->x - m->c.x * v->z;
	temp_matrix.c.z = m->c.x * v->y - m->c.y * v->x;

	//rotate the direction
	//temp_matrix = matrix3f_multiply_by_coefficient(&temp_matrix, -1);

	*m = matrix3f_plus(m, &temp_matrix);
}

void matrix3f_from_euler(Matrix3f *m, float roll, float pitch, float yaw)
{
	float cp = (float)cos(pitch);
	float sp = (float)sin(pitch);
	float sr = (float)sin(roll);
	float cr = (float)cos(roll);
	float sy = (float)sin(yaw);
	float cy = (float)cos(yaw);

	m->a.x = cp * cy;
	m->a.y = (sr * sp * cy) - (cr * sy);
	m->a.z = (cr * sp * cy) + (sr * sy);
	m->b.x = cp * sy;
	m->b.y = (sr * sp * sy) + (cr * cy);
	m->b.z = (cr * sp * sy) - (sr * cy);
	m->c.x = -sp;
	m->c.y = sr * cp;
	m->c.z = cr * cp;
}

void matrix3f_to_euler(Matrix3f *m, float *roll, float *pitch, float *yaw)
{
	if (pitch != NULL) {
		*pitch = -safe_asin(m->c.x);
	}
	if (roll != NULL) {
		*roll = (float)atan2(m->c.y, m->c.z);
	}
	if (yaw != NULL) {
		*yaw = (float)atan2(m->b.x, m->a.x);
	}
}

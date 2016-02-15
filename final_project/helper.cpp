
#include "stdafx.h"
#include "helper.h"

void RtoQ(double a[3][3], double* q)
{
	float trace = a[0][0] + a[1][1] + a[2][2];
	if (trace > 0) {// I changed M_EPSILON to 0
		float s = 0.5f / sqrtf(trace + 1.0f);
		q[3] = 0.25f / s;
		q[0] = (a[2][1] - a[1][2]) * s;
		q[1] = (a[0][2] - a[2][0]) * s;
		q[2] = (a[1][0] - a[0][1]) * s;
	}
	else {
		if (a[0][0] > a[1][1] && a[0][0] > a[2][2]) {
			float s = 2.0f * sqrtf(1.0f + a[0][0] - a[1][1] - a[2][2]);
			q[3] = (a[2][1] - a[1][2]) / s;
			q[0] = 0.25f * s;
			q[1] = (a[0][1] + a[1][0]) / s;
			q[2] = (a[0][2] + a[2][0]) / s;
		}
		else if (a[1][1] > a[2][2]) {
			float s = 2.0f * sqrtf(1.0f + a[1][1] - a[0][0] - a[2][2]);
			q[3] = (a[0][2] - a[2][0]) / s;
			q[0] = (a[0][1] + a[1][0]) / s;
			q[1] = 0.25f * s;
			q[2] = (a[1][2] + a[2][1]) / s;
		}
		else {
			float s = 2.0f * sqrtf(1.0f + a[2][2] - a[0][0] - a[1][1]);
			q[3] = (a[1][0] - a[0][1]) / s;
			q[0] = (a[0][2] + a[2][0]) / s;
			q[1] = (a[1][2] + a[2][1]) / s;
			q[2] = 0.25f * s;
		}
		if (isinf(q[0]))q[0] = 1;
		if (isinf(q[1]))q[1] = 1;
		if (isinf(q[2]))q[2] = 1;
		if (isinf(q[3]))q[3] = 1;
	}
}



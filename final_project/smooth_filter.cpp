
#include "smooth_filter.h"

void SmoothFilter::measurementUpdate()
	{
		K = (P + Q) / (P + Q + R);
		P = R * (P + Q) / (R + P + Q);
	}

float SmoothFilter::update(float measurement)
	{
		measurementUpdate();
		double result = X + (measurement - X) * K;
		X = result;
		// Debug.WriteLine("Measurement " + result + " y: " + y);
		return result;
	}


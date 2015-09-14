class SmoothFilter{
private:
	double Q = 0.000001;
	double R = 0.0001;
	double P = 1, X = 0, K;

	void  measurementUpdate();


public:
	float update(float measurement);
};
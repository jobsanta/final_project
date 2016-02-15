#include <iostream>
#include <Kinect.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/flann/flann.hpp"
#include "hungarian.h"
#include "helper.h"

#include <queue>
#include <vector>
#include <climits>

#include <d3d10.h>
#include <D3DX10.h>
#include <DxErr.h>
#include "d3dUtil.h"

//#include "hungarian.h"

#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3d10.lib")
#pragma comment(lib, "d3dx10.lib")

#include <omp.h>

#include <random>
#define PI 3.14159265

using namespace std;
using namespace cv;
using namespace cv::gpu;

const int num_particle_pso = 32;
const int num_generation_pso = 30;
	
class Tracker
{
public:
	D3DXVECTOR4* handTrack();
	bool headTrack(Point3d &p3d);
	bool detectHand();
	Tracker();
	~Tracker();
	void setKinectParameter(int depthWidth, int depthHeight, int colorWidth, int colorHeight,
		USHORT nMinDistance, USHORT nMaxDistance, UINT16* depthBuffer, RGBQUAD* colorBuffer, ICoordinateMapper* coodinateMapper);
	void reset();


private:

	int nDepthWidth;
	int nDepthHeight;
	int nColorWidth;
	int nColorHeight;
	UINT16* pDepthBuffer;
	RGBQUAD* pColorBuffer;

	USHORT nMinDistance;
	USHORT nMaxDistance;

	float suggestParameter[27];
	float* trackParameter;

	vector<pair<int, float>> a[22500];
	float dis[22500];
	bool vis[22500];

	Mat img_WristMask;
	Mat depthMap;
	Mat depthMap8UC;
	Point rh_d;
	bool firstRun;
	int frame;
	

	CameraSpacePoint* depthToCamera_points;
	ColorSpacePoint* depthToColor_points;
	ICoordinateMapper* m_pCoordinateMapper;
	double minConfig[27];
	double maxConfig[27];

	vector<Point> fingertip_list;
	Mat fingerSegmented[6];
	bool valid_finger[6];
	bool detected_finger[6] ;
	Mat xyFingerSegmented;

	vector<Point3d> fingerDirection;
	vector<Point3d> fingerTipPosition;

	Point3d palmCenter;
	double palmEnergy;
	Point3d palmFaceVector;
	Mat globalTranslate;
	Mat globalRotate;
	Mat globalRotateReverse;

	double m[3][3];

	bool detectedHandRight = false;
	bool detectedPalm = false;

	void dijkstra(int source, int n);
	bool breadfirst(cv::Point root, uchar color, const Mat& img, Mat& out, ushort handCenter_depth);
	bool pixelFill(int point_x, int point_y, uchar color, const Mat& img, Mat& out);
	bool localAngle(int point_x, int point_y, Mat& img);
	bool localMinimum(int point_x, int point_y, Mat& img);
	void floodfill4(int x, int y, int center_x, int center_y, uchar oldColor, uchar newColor, ushort depth, Mat& img, Mat& depthMap);
	void predictFinger();
	void wristDetection();
	void coordinateMapping(UINT16* pDepthBuffer);
	void filterInteractiveArea();

	vector<Point> findExtremePoint(Mat img_hand);
	void findXYFingerSegment(vector<Point> extremePointList, Mat &img_blackwhite, ushort hand_depth, Point2f rh_d);
	void findZFingerSegment(Mat img_depthHand, Point2f rh_d);
	void pcaPalm(Mat img_depthHand, Point2f rh_d);
	void pcaFinger(Point2f rh_d);
	void calculateFingerRotation(Point3d finger_direction, float* rpitch, float* rroll);
	void assignFingerTip(int finger_index, Point3d fingertipDirection, Mat global_rotate, float* suggestPosition);
	float compareHand(float* HandParameter, Mat& depthMap, Point Handcenter, Mat& dist, CameraSpacePoint* HandSurface, D3DXVECTOR4* HandModel, bool optimizing);
	void handEncoding(float* HandParameter, D3DXVECTOR4* output);
	float* optimized(float inputConfig[27], float suggestConfig[27], Point HandPosition_depth, Mat depthMap, Mat dist, CameraSpacePoint* HandSurface, bool UseSuggestion);
	void gradient(D3DXVECTOR4* HandSurface, D3DXVECTOR4* HandOut, float* outputParameter);


};

class comparator //Determines priority for priority queue (shortest edge comes first)
{
public:
	bool operator()(pair<int, float> &a1, pair<int, float> &a2)
	{
		return a1.second > a2.second; //sorting on basis of edge weights
	}
};

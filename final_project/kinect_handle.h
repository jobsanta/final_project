// Kinect Library
#include <Kinect.h>
#include <Kinect.Face.h>

//OpenCV Library
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"

using namespace std;
using namespace cv;
using namespace cv::gpu;

class KinectHandle
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	static const int		particle_flow_height = 180;
	static const int		particle_flow_width = 320;
	static const int		factor = 6;


	// define the face frame features required to be computed by this application
	static const DWORD c_FaceFrameFeatures =
		FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;



public: 
	KinectHandle();
	~KinectHandle();

	HRESULT CreateFirstConnected();
	HRESULT	InitOpenCV();

	// Kinect
	IKinectSensor*     m_pKinectSensor;
	ICoordinateMapper* m_pCoordinateMapper;
	DepthSpacePoint*   m_pDepthCoordinates;

	Mat	referenceFrame;
	Mat Last_u, Last_v;

	// to prevent drawing until we have data for both streams
	bool m_bDepthReceived;
	bool m_bColorReceived;
	bool m_bNearMode;
	bool m_bPaused;

	// Kinect reader
	IColorFrameReader*     m_pColorFrameReader;
	IBodyFrameReader*      m_pBodyFrameReader;
	IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];
	IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];
	IFaceFrameResult*      pFaceFrameResult;
	IMultiSourceFrameReader* m_pMultiSourceFrameReader;

	// For mapping depth to color
	RGBQUAD*	m_pColorRGBX;

	HRESULT	ProcessColor(int nBufferSize);
	HRESULT KinectRead();

	void	RenderParticle();
	void	UpdateParticle(Mat& u, Mat& v);
	void    getFlowField(const Mat& u, const Mat& v, Mat& flowField);
	void    ProcessFrame(INT64 nTime,
			const UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth,
			const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
			const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight, Mat& u, Mat& v);

	void	CloseKinect();


};


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
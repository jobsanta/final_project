
#include <iostream>
#include <queue>
#include <climits>
#include <omp.h>
#include <random>

// Kinect Library
#include <Kinect.h>
#include <Kinect.Face.h>

#include <d3d10.h>
#include <D3DX10.h>
#include <DxErr.h>
#include "d3dUtil.h"
//OpenCV Library
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"

//-- Physx Library
#include <PxPhysicsAPI.h>
#include <PxExtensionsAPI.h>
#include <PxDefaultErrorCallback.h>
#include <PxDefaultAllocator.h>
#include <PxDefaultSimulationFilterShader.h>
#include <PxDefaultCpuDispatcher.h>
#include <PxShapeExt.h>
#include <PxMat33.h>
#include <PxSimpleFactory.h>
#include <vector>
#include <PxVisualDebuggerExt.h>

#include "physxHelper.h"
#include "tracker.h"



#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3d10.lib")
#pragma comment(lib, "d3dx10.lib")

#define PI 3.14159265

using namespace std;
using namespace cv;
using namespace cv::gpu;
using namespace physx;

struct Particle
{
	float x;
	float y;
	float Depth;
};

class KinectHandle
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	static const int		particle_flow_height = 360;
	static const int		particle_flow_width = 640;
	static const int		factor = 3;
	static const int		num_particle_pso = 64;
	static const int		num_generation_pso = 30;
	
	
public: 
	// Kinect
	IKinectSensor*     m_pKinectSensor;
	ICoordinateMapper* m_pCoordinateMapper;
	DepthSpacePoint*   m_pDepthCoordinates;

	Mat	referenceFrame;
	Mat LastGrayFrame;
	Mat Last_u;
	Mat	Last_v;

	PxPhysics* gPhysicsSDK;
	PxScene*   gScene = NULL;

	vector<PxRigidActor*> proxyParticleActor;
	vector<PxRigidActor*> proxyParticleJoint;
	std::vector<Particle> proxyParticle;
	bool gotFace;


	// to prevent drawing until we have data for both streams
	bool m_bDepthReceived;
	bool m_bColorReceived;
	bool m_bNearMode;
	bool m_bPaused;
	bool firstRun;

	// Kinect reader
	IColorFrameReader*     m_pColorFrameReader;
	IBodyFrameReader*      m_pBodyFrameReader;
	IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];
	IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];
	IFaceFrameResult*      pFaceFrameResult;
	IMultiSourceFrameReader* m_pMultiSourceFrameReader;

	// For   mapping depth to color
	RGBQUAD* m_pColorRGBX;
	RGBQUAD* m_pOutputRGBX;
	RGBQUAD* m_pBackgroundRGBX;
	BYTE*	 m_pBodyIndex;


	KinectHandle();
	~KinectHandle();
	
	HRESULT Initialize(PxPhysics* sdk, PxScene* scene);
	HRESULT CreateFirstConnected();
	HRESULT	InitOpenCV();
	void	InitPhysx(PxPhysics* sdk, PxScene* scene);

	HRESULT	ProcessColor(int nBufferSize);
	HRESULT KinectProcess();
	
	void	RenderParticle();
	void	UpdateParticle(Mat& u, Mat& v);
	void    getFlowField(const Mat& u, const Mat& v, Mat& flowField);
	void    ProcessFrame(INT64 nTime,
			UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth,
			RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
			 Mat& u, Mat& v, USHORT nMinDistance, USHORT nMaxDistance);

	void	CloseKinect();
	void	CloseOpenCV();
	void	ParticleCompute();
	BOOLEAN ProcessFaces(IMultiSourceFrame* pMultiFrame);

	HRESULT UpdateBodyData(IBody** ppBodies, IMultiSourceFrame* pMultiFrame);
	
	vector<PxRigidActor*> getProxyParticle();
	void	getFaceResult(float*, float*, float*);


	PxRigidDynamic* CreateSphere(const PxVec3& pos, const PxReal radius, const PxReal density);

private:

	float*                              handParameter;
	LONG                                m_depthWidth;
	LONG                                m_depthHeight;

	LONG                                m_colorWidth;
	LONG                                m_colorHeight;

	LONG                                m_colorToDepthDivisor;

	float								face_x;
	float								face_y;
	float								face_z;

	Tracker* tracker;
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


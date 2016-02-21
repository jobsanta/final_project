#include "stdafx.h"
#include "kinect_handle.h"


//#define USE_PARTICLE true

static const float c_FaceTextLayoutOffsetX = -0.1f;

// face property text layout offset in Y axis
static const float c_FaceTextLayoutOffsetY = -0.125f;

static const float particleSize = 0.1f;
static const float particleRadius = 0.01f;

static const float interact_limit_y = 5.0f; // in 10 CM
static const float interact_limit_y2 = 0.1f; // in 10 CM
static const float interact_limit_Z = -5.0f;


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

#define DBOUT( s )            \
{                             \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}

KinectHandle::KinectHandle()
{
	m_depthWidth = static_cast<LONG>(cDepthWidth);
	m_depthHeight = static_cast<LONG>(cDepthHeight);

	m_colorWidth = static_cast<LONG>(cColorWidth);
	m_colorHeight = static_cast<LONG>(cColorHeight);

	m_colorToDepthDivisor = m_colorWidth / m_depthWidth;

	m_pCoordinateMapper = NULL;
	m_pKinectSensor     = NULL;
	m_pColorFrameReader = NULL;
	m_pBodyFrameReader  = NULL;
	m_pDepthCoordinates = NULL;

	m_pOutputRGBX     = new RGBQUAD[cColorWidth * cColorHeight];
	m_pColorRGBX      = new RGBQUAD[cColorWidth * cColorHeight];
	m_pBackgroundRGBX = new RGBQUAD[cColorWidth * cColorHeight];

	gotFace = false;
	firstRun = true;
	

	// create heap storage for the coorinate mapping from color to depth
	m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];

	tracker = new Tracker();

	for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
	}

	face_x = 0.0f;
	face_y = 2.5f;
	face_z = -8.0f;

	referenceFrame = Mat::zeros(1080, 1920, CV_8UC1);

	proxyParticle.clear();

};

void KinectHandle::InitPhysx(PxPhysics* sdk, PxScene* scene)
{
	gPhysicsSDK = sdk;
	gScene = scene;
}

HRESULT KinectHandle::InitOpenCV()
{
	DWORD colorWidth, colorHeight;
	colorHeight = cColorHeight;
	colorWidth = cColorWidth;

	//Size size(colorWidth, colorHeight);
	//namedWindow("flowfield");
	//////namedWindow("reference");

	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());
	return S_OK;
}

HRESULT KinectHandle::Initialize(PxPhysics* sdk, PxScene* scene)
{
	HRESULT hr;
	hr = InitOpenCV();
	if (SUCCEEDED(hr))
	{

		InitPhysx(sdk, scene);
	}	
	hr = CreateFirstConnected();
	return hr;
}
/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT KinectHandle::CreateFirstConnected()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth |
				FrameSourceTypes::FrameSourceTypes_Color,
				&m_pMultiSourceFrameReader);
		}

		//if (SUCCEEDED(hr))
		//{
		//	// create a face frame source + reader to track each body in the fov
		//	for (int i = 0; i < BODY_COUNT; i++)
		//	{
		//		if (SUCCEEDED(hr))
		//		{
		//			// create the face frame source by specifying the required face frame features
		//			hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
		//		}
		//		if (SUCCEEDED(hr))
		//		{
		//			// open the corresponding reader
		//			hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
		//		}
		//	}
		//}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}

	return hr;


}


HRESULT KinectHandle::KinectProcess()
{
	if (!m_pMultiSourceFrameReader)
	{
		return E_FAIL;
	}

	IMultiSourceFrame* pMultiSourceFrame       = NULL;
	IDepthFrame*       pDepthFrame             = NULL;
	IColorFrame*       pColorFrame             = NULL;
	IBodyIndexFrame*   pBodyIndexFrame         = NULL;
	IBodyFrame*		   pBodyFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	// Get Depth
	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	// Get Color
	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}

		SafeRelease(pColorFrameReference);
	}


	// If all succeeded combine frame
	if (SUCCEEDED(hr))
	{
		INT64                 nDepthTime             = 0;
		IFrameDescription*    pDepthFrameDescription = NULL;
		int                   nDepthWidth            = 0;
		int                   nDepthHeight           = 0;
		USHORT	              nMaxDistance           = 0;
		USHORT                nMinDistance           = 0;
		UINT                  nDepthBufferSize       = 0;
		UINT16                *pDepthBuffer          = NULL;

		IFrameDescription* pColorFrameDescription = NULL;
		int                nColorWidth            = 0;
		int                nColorHeight           = 0;
		ColorImageFormat   imageFormat            = ColorImageFormat_None;
		UINT               nColorBufferSize       = 0;
		RGBQUAD            *pColorBuffer          = NULL;



		pDepthFrame->get_RelativeTime(&nDepthTime);
		pDepthFrame->get_DepthMaxReliableDistance(&nMaxDistance);
		pDepthFrame->get_DepthMinReliableDistance(&nMinDistance);
		hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
		
		if (SUCCEEDED(hr))
		{
			pDepthFrameDescription->get_Width(&nDepthWidth);
			pDepthFrameDescription->get_Height(&nDepthHeight);
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
		}

		// get color frame data
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}
		if (SUCCEEDED(hr))
		{
			pColorFrameDescription->get_Width(&nColorWidth);
			pColorFrameDescription->get_Height(&nColorHeight);
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}


		//Copy color data to the heap
		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}


		if (SUCCEEDED(hr))
		{
			gotFace = FALSE;// ProcessFaces(pMultiSourceFrame);
			ProcessFrame(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight,
				pColorBuffer, nColorWidth, nColorHeight,
				Last_u, Last_v,nMinDistance,nMaxDistance);
		}

		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pMultiSourceFrame);
	
#ifdef USE_PARTICLE 
	ParticleCompute();
#endif
	return hr;

}

void KinectHandle::ProcessFrame(INT64 nTime,
	UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
	Mat&u, Mat&v, USHORT nMinDistance, USHORT nMaxDistance)
{

	if (m_pCoordinateMapper && m_pDepthCoordinates && m_pOutputRGBX &&
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight))
	{
		tracker->setKinectParameter(nDepthWidth, nDepthHeight, nColorWidth, nColorHeight, nMinDistance, nMaxDistance, pDepthBuffer, pColorBuffer, m_pCoordinateMapper);
		
		Point3f headPoint;
		if (tracker->headTrack(headPoint))
		{
			face_x = headPoint.x*10.0;
			face_y = headPoint.y*10.0;
			face_z = headPoint.z*10.0;
		}
		DepthSpacePoint headPoint_depth;


		int boxSize = 100;
		int halfBoxSize = boxSize / 2.0f;



		
#ifndef USE_PARTICLE


		////Simple Background subtraction, only the object in the interactive volume is considered
		//for (int j = 0; j < nDepthHeight; j++)
		//{
		//	ushort* p = depthMap.ptr<ushort>(j);
		//	float* q = img0.ptr<float>(j);
		//	for (int i = 0; i < nDepthWidth; i++)
		//	{
		//		float y = cos(degree * PI / 180.0f)*points[j*nDepthWidth + i].Y - sin(degree * PI / 180.0f)*points[j*nDepthWidth + i].Z + height;
		//		float z = -sin(degree * PI / 180.0f)*points[j*nDepthWidth + i].Y - cos(degree * PI / 180.0f)*points[j*nDepthWidth + i].Z;

		//		if (y < 0.05 || z < -0.5)
		//		{
		//			p[i] = 0;
		//			q[i] = 0;
		//		}
		//	}
		//}
		//if (gotHandRight && img0.at<float>(rh_d.Y, rh_d.X) ==0)
		//	gotHandRight = false;


		//float* suggestPosition = new float[7];

		//suggestPosition[0] = rh.X - 0.02;
		//suggestPosition[1] = cos(degree * PI / 180.0f)*rh.Y - sin(degree * PI / 180.0f)*rh.Z + height - 0.01;
		//suggestPosition[2] = -sin(degree * PI / 180.0f)*rh.Y - cos(degree * PI / 180.0f)*rh.Z + 0.02;
		//suggestPosition[3] = temp.x;
		//suggestPosition[4] = temp.y;
		//suggestPosition[5] = temp.z;
		//suggestPosition[6] = temp.w;

		//if (firstRun)
		//{
		//	handParameter[0] = suggestPosition[0];
		//	handParameter[1] = suggestPosition[1];
		//	handParameter[2] = suggestPosition[2];
		//	firstRun = false;
		//}

		//std::free(points);

		//Mat img1, segmented;
		//img1.create(img0.rows, img0.cols, CV_32FC1);
		//segmented.create(img0.rows, img0.cols, CV_8UC1);
		//cv::bilateralFilter(img0, img1, 3, 50, 50);
		//img1.convertTo(segmented, CV_8UC1, 255);
		//float result = 99999999;

		if (tracker->detectHand())
		{
			DBOUT("\n got hand right");
			////Compute Distance transform
			//DepthSpacePoint* depthPoints = new DepthSpacePoint[48];
			D3DXVECTOR4* output = new D3DXVECTOR4[48];
			output = tracker->handTrack();
			//tracker->handTrack();
			//CameraSpacePoint* spacePoint = new CameraSpacePoint[48];
			//Mat bw, dist;
			//cv::threshold(segmented, bw, 100, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
			//cv::distanceTransform(bw, dist, CV_DIST_L2, 3);


			////Random point used in optimization
			//for (int i = 0; i < 256; i++)
			//{
			//	DepthSpacePoint random_surface;
			//	UINT16 depth;
			//	do
			//	{
			//		DBOUT("\n stuck in the loop");
			//		float random_x = (((float)rand() / (float)(RAND_MAX)) * 150.0) - 75.0f;
			//		float random_y = (((float)rand() / (float)(RAND_MAX)) * 150.0f) - 75.0f;
			//		random_surface.Y = rh_d.Y + random_y;
			//		random_surface.X = rh_d.X + random_x;
			//		depth = depthMap.at<ushort>(random_surface.Y, random_surface.X);

			//	} while (depth == 0);
			//	//circle(segmented, cv::Point(random_surface.X, random_surface.Y), 1, cv::Scalar(255), -1);
			//	m_pCoordinateMapper->MapDepthPointToCameraSpace(random_surface, depth, &camera_surface[i]);
			//}
			//DBOUT("\n start optimized");
			//handParameter = optimized(handParameter, suggestPosition, cv::Point(rh_d.X, rh_d.Y), rh, depthMap, dist, camera_surface);
			//HandEncoding(handParameter, output);
			//DBOUT("\n optimized");

			//If not previously created, create it
			if (proxyParticle.empty())
			{
				DBOUT("\n first run");
				//loop thought hand sphere
				for (int i = 0; i < 48; i++)
				{
					output[i].x *= 10.0f;
					output[i].y *= 10.0f;
					output[i].z *= 10.0f;
					Particle newParticle = { output[i].x, output[i].y, output[i].z };
					PxRigidDynamic* newParticleActor = CreateSphere(PxVec3(output[i].x, output[i].y, output[i].z), particleSize, 1.0f);
					newParticleActor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
	

					PxU32 nShapes = newParticleActor->getNbShapes();
					PxShape** shapes = new PxShape*[nShapes];
					newParticleActor->getShapes(shapes, nShapes);
					
					while (nShapes--)
					{
						shapes[nShapes]->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
					}

					proxyParticle.push_back(newParticle);
					proxyParticleActor.push_back(newParticleActor);

					

					PxRigidDynamic* newParticleActorJoint = CreateSphere(PxVec3(output[i].x, output[i].y, output[i].z), particleSize, 1.0f);
					((PxRigidBody*)newParticleActorJoint)->setMass(1);
					newParticleActorJoint->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, true);

					if (i >= 16 && i <= 21)
					setupFiltering(newParticleActorJoint, FilterGroup::eInteract, FilterGroup::ePARTICLE | FilterGroup::eInteract);
					else
					{
						setupFiltering(newParticleActorJoint, FilterGroup::ePARTICLE, FilterGroup::eInteract | FilterGroup:: ePARTICLE);
					}


					PxDistanceJoint* joint = PxDistanceJointCreate(*gPhysicsSDK, newParticleActor, PxTransform(PxVec3(0.0, 0.0, 0.0)), newParticleActorJoint, PxTransform(PxVec3(0.0, 0.0, 0.0)));
					joint->setMaxDistance(0.01f);
					//joint->setDamping(0.5);
					joint->setStiffness(1000.0f);
					joint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
					joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);

					//PxFixedJoint* joint = PxFixedJointCreate(*gPhysicsSDK, newParticleActor, PxTransform(PxVec3(0.0, 0.0, 0.0)), newParticleActorJoint, PxTransform(PxVec3(0.0, 0.0, 0.0)));

					proxyParticleJoint.push_back(newParticleActorJoint);
					

				}
			}
			else // otherwise updated it
			{
				//DBOUT("\n updated");
				for (int i = 0; i < 48; i++)
				{
					output[i].x *= 10.0f;
					output[i].y *= 10.0f;
					output[i].z *= 10.0f;
					proxyParticle[i] = { output[i].x, output[i].y, output[i].z };
					PxTransform transform_update(PxVec3(output[i].x, output[i].y, output[i].z), PxQuat::createIdentity());
					((PxRigidDynamic*)proxyParticleActor[i])->setKinematicTarget(transform_update);
					((PxRigidDynamic*)proxyParticleJoint[i])->setGlobalPose(transform_update);
				}
			}
			
			//std::free(suggestPosition);
			//std::free(depthPoints);
			//std::free(output);
			//std::free(spacePoint);
			
		}
		else // no hand detected, clear 
		{
			if (!proxyParticleActor.empty())
				for (int i = 0; i < 48; i++)
				{
					proxyParticleActor[i]->release();
					proxyParticleJoint[i]->release();
				}

			proxyParticleJoint.clear();
			proxyParticle.clear();
			proxyParticleActor.clear();

			tracker->reset();
		}
#endif
		
		//if (gotFace)
			//{
			//	PointF facePoints[FacePointType::FacePointType_Count];
			//	pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
			//	float average_y = (facePoints[FacePointType::FacePointType_EyeLeft].Y + facePoints[FacePointType::FacePointType_EyeRight].Y) / 2.0f;
			//	float average_x = (facePoints[FacePointType::FacePointType_EyeLeft].X + facePoints[FacePointType::FacePointType_EyeRight].X) / 2.0f;
			//	if (average_x > 0 && average_x < 1980 && average_y > 0 && average_y < 1080)
			//	{
			//		int index = average_y* nColorWidth + average_x;
			//		
			//		DepthSpacePoint p = m_pDepthCoordinates[index];
			//		//	 Values that are negative infinity means it is an invalid color to depth mapping so we
			//		//	 skip processing for this pixel
			//		if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
			//		{
			//			int depthX = static_cast<int>(p.X + 0.5f);
			//			int depthY = static_cast<int>(p.Y + 0.5f);
			//			if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
			//			{
			//				//if the position is valid update the depth of the particle.
			//				BYTE player = pBodyIndexBuffer[depthX + (depthY * cDepthWidth)];
			//				UINT16 depth = pDepthBuffer[depthX + depthY*cDepthWidth];
			//				if (player != 0xff && depth > 500 && depth < 2000)
			//				{
			//					CameraSpacePoint camPoint;
			//					m_pCoordinateMapper->MapDepthPointToCameraSpace(p, depth, &camPoint);
			//					face_x = camPoint.X;//-2.0f + 4.0f*x / (float)cColorWidth;
			//					face_y = cos(degree * PI / 180.0f)*camPoint.Y - sin(degree * PI / 180.0f)*camPoint.Z + height;
			//					face_z = sin(degree * PI / 180.0f)*camPoint.Y + cos(degree * PI / 180.0f)*camPoint.Z;
			//					//	Particle newParticle = { x, y, depth };
			//					face_x *= 10.0f;
			//					face_y *= 10.0f;
			//					face_z *= -10.0f;
			//				}
			//			}
			//		}
			//	}

#ifdef USE_PARTICLE

#pragma region update_particle
			if (!u.empty() && !v.empty())
			{
				vector<Particle>::iterator it = proxyParticle.begin();
				vector<PxRigidActor*>::iterator itact = proxyParticleActor.begin();
				vector<PxRigidActor*>::iterator itjoint = proxyParticleJoint.begin();

				//for (int i = 0; i < boxes.size(); i++)
				//{
				//	((PxRigidDynamic*)boxes[i])->wakeUp();
				//}

				for (; it != proxyParticle.end();)
				{

					int x_small = (*it).x / factor;
					int y_small = (*it).y / factor;

					if (x_small >= 0 && x_small < 640 && y_small >= 0 && y_small < 360)
					{

						//Calculate position of existing particle using acquire particle flow value
						const float* ptr_u = u.ptr<float>(y_small);
						const float* ptr_v = v.ptr<float>(y_small);
						float dx = ptr_u[x_small];
						float dy = ptr_v[x_small];
						(*it).x += 2*factor * dx;
						(*it).y += 2*factor * dy;
						int x = (*it).x;
						int y = (*it).y;
						double depth = (*it).Depth;
						//float x_r = -2.0f + 4.0f*x / (float)cColorWidth;
						//float y_r = 2.0f * (-y / (float)cColorHeight) + 1.0f;


						int index = y*nColorWidth + x;
						if (x < 0 || x >= 1920 || y < 0 || y >= 1080)
						{
							it = proxyParticle.erase(it);
							(*itact)->release();
							itact = proxyParticleActor.erase(itact);
							(*itjoint)->release();
							itjoint = proxyParticleJoint.erase(itjoint);


						}
						else
						{
							DepthSpacePoint p = m_pDepthCoordinates[index];

							//	 Values that are negative infinity means it is an invalid color to depth mapping so we
							//	 skip processing for this pixel
							if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
							{
								int depthX = static_cast<int>(p.X + 0.5f);
								int depthY = static_cast<int>(p.Y + 0.5f);

								if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
								{
									//if the position is valid update the depth of the particle.
									//BYTE player = pBodyIndexBuffer[depthX + (depthY * cDepthWidth)];
									UINT16 depth = pDepthBuffer[depthX + depthY*cDepthWidth];

									//if (player != 0xff && depth > 500 && depth < 2000)
									if (depth > 500 && depth < 2000)
									{
										CameraSpacePoint camPoint;
										m_pCoordinateMapper->MapDepthPointToCameraSpace(p, depth, &camPoint);


										float x_r = camPoint.X;//-2.0f + 4.0f*x / (float)cColorWidth;
										float y_r = cos(degree * PI / 180.0f)*camPoint.Y - sin(degree * PI / 180.0f)*camPoint.Z + height;
										float depthM = sin(degree * PI / 180.0f)*camPoint.Y + cos(degree * PI / 180.0f)*camPoint.Z;

										//	Particle newParticle = { x, y, depth };

										x_r *= 10.0f;
										y_r *= 10.0f;
										depthM *= -10.0f;


										PxTransform transform_update(PxVec3(x_r, y_r, depthM), PxQuat::createIdentity());
										((PxRigidDynamic*)(*itact))->setKinematicTarget(transform_update);
										

										//PxRigidDynamic* newParticleActor = CreateSphere(PxVec3(x_r, y_r, depthM), depthM/5.0, 1.0f);
										//newParticleActor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
										//(*itact)->release();
										//(*itact) = newParticleActor;

										++it;
										++itact;
										++itjoint;

										//circle(referenceFrame, Point(x, y), particleSize*10.0f, Scalar(255), -1);

									}
									else
									{
										it = proxyParticle.erase(it);
										(*itact)->release();
										itact = proxyParticleActor.erase(itact);
										(*itjoint)->release();
										itjoint = proxyParticleJoint.erase(itjoint);
									}

								}
								else
								{
									it = proxyParticle.erase(it);
									(*itact)->release();
									itact = proxyParticleActor.erase(itact);
									(*itjoint)->release();
									itjoint = proxyParticleJoint.erase(itjoint);
								}

							}
							else
							{
								it = proxyParticle.erase(it);
								(*itact)->release();
								itact = proxyParticleActor.erase(itact);
								(*itjoint)->release();
								itjoint = proxyParticleJoint.erase(itjoint);
							}
						}
					}
				}



			}

#pragma endregion
			// loop over output pixels
#pragma region New_particle
			for (int colorIndex = 0; colorIndex < (nColorWidth*nColorHeight); ++colorIndex)
			{
				// default setting source to copy from the background pixel
				const RGBQUAD* pSrc = m_pBackgroundRGBX + colorIndex;

				DepthSpacePoint p = m_pDepthCoordinates[colorIndex];

				// Values that are negative infinity means it is an invalid color to depth mapping so we
				// skip processing for this pixel
				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
				{
					int depthX = static_cast<int>(p.X + 0.5f);
					int depthY = static_cast<int>(p.Y + 0.5f);

					if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
					{
						//BYTE player = pBodyIndexBuffer[depthX + (depthY * cDepthWidth)];
						UINT16 depth = pDepthBuffer[depthX + depthY*cDepthWidth];

						// if we're tracking a player for the current pixel, draw from the color camera
						if (depth > 500 && depth < 2000)
						{
							// set source for copy to the color pixels
							pSrc = m_pColorRGBX + colorIndex;
							int x = colorIndex % cColorWidth;
							int y = colorIndex / cColorWidth;
							//if (x % 5 == 0 && y % 5 == 0 && referenceFrame.at<UCHAR>(y, x) == 0)
							if(x%5 ==0 && y%5 ==0)
							{
								//float depthM = (float)(depth - 500) / 50.0;

								CameraSpacePoint camPoint;
								m_pCoordinateMapper->MapDepthPointToCameraSpace(p, depth, &camPoint);
	/*							float degree = 18.0f + (float)(cColorHeight - y) / (float)cColorHeight * 60.0f;
								float y_r = height - cos(degree * PI / 180.0f)*depth / 1000.0f;
								float depthM = -sin(degree * PI / 180.0f)*depth / 100.0f;*/


								float x_r = camPoint.X;//-2.0f + 4.0f*x / (float)cColorWidth;
								float y_r = cos(degree * PI / 180.0f)*camPoint.Y - sin(degree * PI / 180.0f)*camPoint.Z + height;
								float depthM = sin(degree * PI / 180.0f)*camPoint.Y + cos(degree * PI / 180.0f)*camPoint.Z;

								Particle newParticle = { x, y, depth };

								//float y_r = 2.0f * (-y / (float)cColorHeight) + 1.0f;

								x_r *= 10.0f;
								y_r *= 10.0f;
								depthM *= -10.0f;

								PxOverlapBuffer hit;
								PxTransform shapePose = PxTransform(PxVec3(x_r, y_r, depthM));
								PxQueryFilterData fd;
								fd.flags |= PxQueryFlag::eANY_HIT;


								if (!gScene->overlap(PxSphereGeometry(particleSize), shapePose, hit, fd) && y_r > interact_limit_y2 &&y_r < interact_limit_y && depthM > interact_limit_Z)
								{
									proxyParticle.push_back(newParticle);
									PxRigidDynamic* newParticleActor = CreateSphere(PxVec3(x_r, y_r, depthM), particleSize, 1.0f);
									newParticleActor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);

									PxU32 nShapes = newParticleActor->getNbShapes();
									PxShape** shapes = new PxShape*[nShapes];
									newParticleActor->getShapes(shapes, nShapes);

									while (nShapes--)
									{
										shapes[nShapes]->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
									}

									PxRigidDynamic* newParticleActorJoint = CreateSphere(PxVec3(x_r, y_r, depthM), particleSize, 1.0f);
									newParticleActorJoint->setActorFlag(PxActorFlag::eDISABLE_GRAVITY,true);

									setupFiltering(newParticleActorJoint, FilterGroup::ePARTICLE, FilterGroup::ePARTICLE);

									PxDistanceJoint* joint = PxDistanceJointCreate(*gPhysicsSDK, newParticleActor, PxTransform(PxVec3(0.0, 0.0, 0.0)), newParticleActorJoint, PxTransform(PxVec3(0.0, 0.0, 0.0)));
									joint->setDamping(100.0);
									joint->setStiffness(10000.0f);
									joint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
									proxyParticleActor.push_back(newParticleActor);
									proxyParticleJoint.push_back(newParticleActorJoint);


								}

							}
						}
					}
				}


				// write output
				m_pOutputRGBX[colorIndex] = *pSrc;
			}
#pragma endregion
			
			//std::vector<Particle> temporary;
			//std::vector<PxRigidActor*> tempAct;
			//for (int i = 0; i < proxyParticle.size(); i += 4)
			//{
			//	PxTransform trans = ((PxRigidDynamic*)(proxyParticleActor[i]))->getGlobalPose();
			//	const PxU32 buffersize = 256;
			//	PxOverlapHit overlapBuffer[buffersize];
			//	PxOverlapBuffer buf(overlapBuffer, buffersize);
			//	if (gScene->overlap(PxSphereGeometry(particleSize*3.0), trans, buf))
			//	{
			//		int hit = buf.nbTouches;
			//		if (hit > 27)
			//		{
			//			temporary.push_back((proxyParticle[i]));
			//			PxRigidDynamic* newParticleActor = CreateSphere(PxVec3(trans.p.x, trans.p.y, trans.p.z - 0.4), particleSize*3.0, 1.0f);
			//			newParticleActor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
			//			tempAct.push_back(newParticleActor);
			//		}
			//		else if (gScene->overlap(PxSphereGeometry(particleSize*2.0), trans, buf))
			//		{
			//			int hit = buf.nbTouches;
			//			if (hit > 8)
			//			{
			//				temporary.push_back((proxyParticle[i]));
			//				PxRigidDynamic* newParticleActor = CreateSphere(PxVec3(trans.p.x, trans.p.y, trans.p.z - 0.2), particleSize*2.0, 1.0f);
			//				newParticleActor->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
			//				tempAct.push_back(newParticleActor);
			//			}
			//		}
			//	}
			//}
			//proxyParticle.insert(proxyParticle.end(), temporary.begin(), temporary.end());
			//proxyParticleActor.insert(proxyParticleActor.end(), tempAct.begin(), tempAct.end());

#pragma region ClearOverlapparticle
			vector<Particle>::iterator it = proxyParticle.begin();
			vector<PxRigidActor*>::iterator itact = proxyParticleActor.begin();
			vector<PxRigidActor*>::iterator itjoint = proxyParticleJoint.begin();



			for (; it != proxyParticle.end();)
			{
				PxShape* buffer[2];
				PxShape* bufferAct[2];
				((PxRigidDynamic*)(*itact))->getShapes(buffer, 2);

				buffer[0]->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);

				((PxRigidDynamic*)(*itjoint))->getShapes(bufferAct, 2);

				bufferAct[0]->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);

				PxTransform trans = ((PxRigidDynamic*)(*itact))->getGlobalPose();


				PxOverlapBuffer hit;
				PxTransform shapePose = trans;
				PxQueryFilterData fd;
				fd.flags |= PxQueryFlag::eANY_HIT;

				//gScene->overlap(PxSphereGeometry(0.01f), shapePose, hit, fd) ||

				if ( trans.p.y > interact_limit_y || trans.p.y < interact_limit_y2 || trans.p.z < interact_limit_Z)
				{
					it = proxyParticle.erase(it);
					(*itact)->release();
					itact = proxyParticleActor.erase(itact);
					(*itjoint)->release();
					itjoint = proxyParticleJoint.erase(itjoint);
				}
				else
				{
					buffer[0]->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
					bufferAct[0]->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
					++it;
					++itact;
					++itjoint;
				}

			}
#pragma endregion
#endif
		
	}
}

void KinectHandle::ParticleCompute()
{

	referenceFrame = Mat::zeros(1080, 1920, CV_8UC1);
	Mat m_colorMat(cColorHeight, cColorWidth, CV_8UC4, reinterpret_cast<void*>(m_pOutputRGBX));
	Mat m_color_small, m_color_float, frame1Gray;
	resize(m_colorMat, m_color_small, Size(particle_flow_width, particle_flow_height));
	m_color_small.convertTo(m_color_float, CV_32F, 1.0 / 255.0);

	cvtColor(m_color_float, frame1Gray, COLOR_BGR2GRAY);

	if (!LastGrayFrame.empty())
	{
		GpuMat d_frame0(LastGrayFrame);
		GpuMat d_frame1(frame1Gray);

		BroxOpticalFlow d_flow(0.197, 50.0, 0.75, 1, 14, 8);

		GpuMat d_fu, d_fv;
		d_flow(d_frame0, d_frame1, d_fu, d_fv);

		Last_u = Mat(d_fu).clone();
		Last_v = Mat(d_fv).clone();

		//Mat flowFieldForward = m_color_small.clone();
		//getFlowField(Mat(d_fu), Mat(d_fv), flowFieldForward);
		//imshow("flowfield", flowFieldForward);
	}
	LastGrayFrame = frame1Gray.clone();
}

void KinectHandle::getFlowField(const Mat& u, const Mat& v, Mat& flowField)
{
	float maxDisplacement = -1.0f;

	for (int i = 0; i < u.rows; i += 10)
	{
		const float* ptr_u = u.ptr<float>(i);
		const float* ptr_v = v.ptr<float>(i);

		for (int j = 0; j < u.cols; j += 10)
		{
			float d = sqrt(ptr_u[j] * ptr_u[j] + ptr_v[j] * ptr_v[j]); //max(fabsf(ptr_u[j]), fabsf(ptr_v[j]));

			if (d > maxDisplacement)
				maxDisplacement = d;
		}
	}

	//flowField = Mat::zeros(u.size(), CV_8UC3);


	for (int i = 0; i < flowField.rows; i += 10)
	{
		const float* ptr_u = u.ptr<float>(i);
		const float* ptr_v = v.ptr<float>(i);

		//Vec4b* row = flowField.ptr<Vec4b>(i);

		for (int j = 0; j < flowField.cols; j += 10)
		{
			//row[j][0] = 0;
			//row[j][1] = static_cast<unsigned char> (mapValue(-ptr_v[j], -maxDisplacement, maxDisplacement, 0.0f, 255.0f));
			//row[j][2] = static_cast<unsigned char> (mapValue(ptr_u[j], -maxDisplacement, maxDisplacement, 0.0f, 255.0f));
			//row[j][3] = 255;


			Point p = Point(j, i);
			float l = sqrt(ptr_u[j] * ptr_u[j] + ptr_v[j] * ptr_v[j]);
			float l_max = maxDisplacement;

			float dx = ptr_u[j];
			float dy = ptr_v[j];
			if (l > 0 && flowField.at<Vec4b>(i, j) != Vec4b(255, 0, 0))
			{
				double spinSize = 5.0 * l / l_max;  // Factor to normalise the size of the spin depeding on the length of the arrow

				Point p2 = Point(p.x + (int)(dx), p.y + (int)(dy));
				line(flowField, p, p2, CV_RGB(0, 255, 0), 1);

				double angle;               // Draws the spin of the arrow
				angle = atan2((double)p.y - p2.y, (double)p.x - p2.x);

				p.x = (int)(p2.x + spinSize * cos(angle + 3.1416 / 4));
				p.y = (int)(p2.y + spinSize * sin(angle + 3.1416 / 4));
				line(flowField, p, p2, CV_RGB(0, 255, 0), 1);

				p.x = (int)(p2.x + spinSize * cos(angle - 3.1416 / 4));
				p.y = (int)(p2.y + spinSize * sin(angle - 3.1416 / 4));
				line(flowField, p, p2, CV_RGB(0, 255, 0), 1);

			}
		}
	}

}

PxRigidDynamic* KinectHandle::CreateSphere(const PxVec3& pos, const PxReal radius, const PxReal density)
{
	PxTransform transform(pos, PxQuat::createIdentity());
	PxSphereGeometry geometry(radius);

	PxMaterial* mMaterial = gPhysicsSDK->createMaterial(1.0, 1.0, 1.0);

	PxRigidDynamic* actor = PxCreateDynamic(*gPhysicsSDK, transform, geometry, *mMaterial, density);

	if (!actor)
		cerr << "create actor failed" << endl;
	actor->setAngularDamping(0.75);
	actor->setLinearVelocity(PxVec3(0, 0, 0));
	gScene->addActor(*actor);
	return actor;
}

vector<PxRigidActor*> KinectHandle::getProxyParticle()
{
	return proxyParticleActor;
}

KinectHandle::~KinectHandle()
{
	CloseKinect();
	CloseOpenCV();
};

void KinectHandle::CloseKinect()
{
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();

	}
	SafeRelease(m_pKinectSensor);

	// done with face sources and readers
	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
	}

	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with color frame reader
	SafeRelease(m_pColorFrameReader);

	//SAFE_RELEASE(m_pFaceTracker);
	//SAFE_RELEASE(m_pFTResult);

	if (m_pOutputRGBX)
	{
		delete[] m_pOutputRGBX;
		m_pOutputRGBX = NULL;
	}

	if (m_pBackgroundRGBX)
	{
		delete[] m_pBackgroundRGBX;
		m_pBackgroundRGBX = NULL;
	}

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	if (m_pDepthCoordinates)
	{
		delete[] m_pDepthCoordinates;
		m_pDepthCoordinates = NULL;
	}

}

void KinectHandle::CloseOpenCV()
{
	destroyAllWindows();
}

BOOLEAN KinectHandle::ProcessFaces(IMultiSourceFrame* pMultiFrame)
{
	HRESULT hr;
	IBody* ppBodies[BODY_COUNT] = { 0 };
	bool bHaveBodyData = SUCCEEDED(UpdateBodyData(ppBodies, pMultiFrame));
	bool bOneFaceTraked = false;
	// iterate through each face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
			if (bFaceTracked)
			{
				bOneFaceTraked = true;
			}

		}

		if (SUCCEEDED(hr))
		{
			if (bFaceTracked)
			{
				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);
			}
			else
			{
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				if (bHaveBodyData)
				{
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
					IBody* pBody = ppBodies[iFace];
					if (pBody != nullptr)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if (SUCCEEDED(hr) && bTracked)
						{
							// get the tracking ID of this body
							hr = pBody->get_TrackingId(&bodyTId);
							if (SUCCEEDED(hr))
							{
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
							}
						}
					}
				}
			}
		}

		SafeRelease(pFaceFrame);
	}

	if (bHaveBodyData)
	{
		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}
	return bOneFaceTraked;
}

HRESULT KinectHandle::UpdateBodyData(IBody** ppBodies, IMultiSourceFrame* pMultiFrame)
{
	HRESULT hr = E_FAIL;

	if (pMultiFrame != nullptr)
	{
		IBodyFrame* pBodyFrame = nullptr;
		IBodyFrameReference* pBodyFrameReference = nullptr;
		hr = pMultiFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
			}
		}

		SafeRelease(pBodyFrame);
	}

	return hr;
}

void KinectHandle::getFaceResult(float* x, float* y , float* z)
{
	if (firstRun)
	{
		*x = 0.0;
		*z = -5;
		*y = 2.5f;
		firstRun = false;
	}
	else
	{
		*x = face_x;
		*y = face_y;
		*z = face_z;
	}

}



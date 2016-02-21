#include "stdafx.h"
#include "tracker.h"
#include <sstream>
//#inlcude "helper.h"

#define INF INT_MAX


#define DBOUT( s )            \
{                             \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}

static const float particleSize = 0.2f;
static const float particleRadius = 0.01f;

static const float height = 0.9f;
static const float degree = 40.0;
static const float interact_limit_y = 5.0f; // in 10 CM
static const float interact_limit_y2 = 0.1f; // in 10 CM
static const float interact_limit_Z = -5.0f;


static const int        cDepthWidth = 512;
static const int        cDepthHeight = 424;
static const int        cColorWidth = 1920;
static const int        cColorHeight = 1080;

Tracker::Tracker()
{
	trackParameter = new float[27];
	firstRun = true;
	frame = 0;

	cos_deg = cos(degree*PI / 180.0f);
	sin_deg = sin(degree*PI / 180.0f);
	tan_deg = tan(degree*PI / 180.0f);

	for (int i = 0; i < 27; i++)
	{
		trackParameter[i] = 0;
	}
	D3DXQUATERNION temp;
	D3DXQuaternionRotationYawPitchRoll(&temp, 0.0f, D3DX_PI / 2.0f, 0.0f);
	trackParameter[3] = temp.x;
	trackParameter[4] = temp.y;
	trackParameter[5] = temp.z;
	trackParameter[6] = temp.w;

	depthToCamera_points = new CameraSpacePoint[cDepthWidth*cDepthHeight];
	depthToColor_points = new ColorSpacePoint[cColorWidth* cColorHeight];

	//Orientation
	maxConfig[3] = 1.0f;
	minConfig[3] = -1.0f;
	maxConfig[4] = 1.0f;
	minConfig[4] = -1.0f;
	maxConfig[5] = 1.0f;
	minConfig[5] = -1.0f;
	maxConfig[6] = 1.0f;
	minConfig[6] = -1.0f;

	//index_finger
	maxConfig[7] = D3DX_PI / 2.0f;
	minConfig[7] = 0;//-D3DX_PI/2.0f;
	maxConfig[8] = 0.2636f;
	minConfig[8] = -0.2636f;
	maxConfig[9] = D3DX_PI / 2.0f;
	minConfig[9] = 0;//-D3DX_PI / 2.0f;
	maxConfig[10] = D3DX_PI / 2.0f;
	minConfig[10] = 0;//-D3DX_PI / 2.0f;

	//Middle Finger
	maxConfig[11] = D3DX_PI / 2.0f;
	minConfig[11] = 0;//-D3DX_PI / 2.0f;
	maxConfig[12] = 0.2636f;
	minConfig[12] = -0.2636f;
	maxConfig[13] = D3DX_PI / 2.0f;
	minConfig[13] = 0;//-D3DX_PI / 2.0f;
	maxConfig[14] = D3DX_PI / 2.0f;
	minConfig[14] = 0;//-D3DX_PI / 2.0f;

	//Ring Finger
	maxConfig[15] = D3DX_PI / 2.0f;
	minConfig[15] = 0;//-D3DX_PI / 2.0f;
	maxConfig[16] = 0.2636f;
	minConfig[16] = -0.2636f;
	maxConfig[17] = D3DX_PI / 2.0f;
	minConfig[17] = 0;//-D3DX_PI / 2.0f;
	maxConfig[18] = D3DX_PI / 2.0f;
	minConfig[18] = 0;//-D3DX_PI / 2.0f;

	//Pinky Finger
	maxConfig[19] = D3DX_PI / 2.0f;
	minConfig[19] = 0;//-D3DX_PI / 2.0f;
	maxConfig[20] = 0;//0.5236f;
	minConfig[20] = -D3DX_PI / 2.0f;
	maxConfig[21] = D3DX_PI / 2.0f;
	minConfig[21] = 0;//-D3DX_PI / 2.0f;
	maxConfig[22] = D3DX_PI / 2.0f;
	minConfig[22] = 0;//-D3DX_PI / 2.0f;

	maxConfig[23] = D3DX_PI / 2.0f;
	minConfig[23] = -D3DX_PI / 2.0f;
	minConfig[24] = 0;// -D3DX_PI / 2.0f;
	maxConfig[24] = D3DX_PI / 2.0f;

	minConfig[25] = 0;//-D3DX_PI / 2.0f;
	maxConfig[25] = D3DX_PI / 2.0f;

	minConfig[26] = 0;//-D3DX_PI / 2.0f;
	maxConfig[26] = D3DX_PI / 2.0f;

}

Tracker::~Tracker()
{

}


bool Tracker::detectHand()
{
	wristDetection();
	filterInteractiveArea();

	return detectedHandRight;


}

void Tracker::reset()
{
	firstRun = true;
	frame = 0;
}

D3DXVECTOR4* Tracker::handTrack()
{
	//Add Wrist Detection here


	Mat img_blackwhite_inv, dist, img_blackwhite, color;
	cv::threshold(depthMap8UC, img_blackwhite_inv, 100, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
	distanceTransform(img_blackwhite_inv, dist, CV_DIST_L2, 3);
	cv::threshold(depthMap8UC(Rect(rh_d.x - 75, rh_d.y - 75, 150, 150)), img_blackwhite, 100, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	cv::cvtColor(depthMap8UC, color, CV_GRAY2BGR);

	detectedPalm = false;
	if (frame % 1 == 0)
	{	//Clear Previous value
		for (int i = 0; i < 6; i++)
		{
			valid_finger[i] = false;
			detected_finger[i] = false;
		}

		fingerDirection.clear();
		fingerTipPosition.clear();
		fingertip_list.clear();

		vector<Point> extremePoints;
		extremePoints = findExtremePoint(img_blackwhite);
		ushort hand_depth = depthMap.at<ushort>(rh_d.y, rh_d.x);


		findXYFingerSegment(extremePoints, img_blackwhite, hand_depth, rh_d);
		Mat img_hand = depthMap(Rect(rh_d.x - 75, rh_d.y - 75, 150, 150));
		findZFingerSegment(img_hand, rh_d);

		for (int i = 0; i < extremePoints.size(); i++)
			circle(xyFingerSegmented, extremePoints[i], 3, Scalar(128));
		imshow("xyfinger", xyFingerSegmented);

		pcaPalm(img_hand, rh_d, color);// need to return palm orientation


		pcaFinger(rh_d,color); // need to return
		predictFinger();
	}
	frame++;
	bool useSuggestion = detectedPalm && (detected_finger[0] || detected_finger[1] || detected_finger[2] || detected_finger[3] || detected_finger[4] || detected_finger[5]);
	if (firstRun)
	{
		if (useSuggestion)
		{
			std::copy(&suggestParameter[0], &suggestParameter[26], trackParameter);
			firstRun = false;
		}
	}


	D3DXVECTOR4* output = new D3DXVECTOR4[48];
	if (!firstRun)
	{
		CameraSpacePoint camera_surface[256];
		DepthSpacePoint random_surface[256];
		UINT16 depth[256];
		for (int i = 0; i < 256; i++)
		{
			do
			{
				float random_x = (((float)rand() / (float)(RAND_MAX)) * 150.0) - 75.0f;
				float random_y = (((float)rand() / (float)(RAND_MAX)) * 150.0f) - 75.0f;
				random_surface[i].Y = rh_d.y + random_y;
				random_surface[i].X = rh_d.x + random_x;
				depth[i] = depthMap.at<ushort>(random_surface[i].Y, random_surface[i].X);
			} while (depth[i] == 0);
			//circle(segmented, cv::Point(random_surface.X, random_surface.Y), 1, cv::Scalar(255), -1);
			//m_pCoordinateMapper->MapDepthPointToCameraSpace(random_surface, depth, &camera_surface[i]);.
		}
		m_pCoordinateMapper->MapDepthPointsToCameraSpace(256, random_surface, 256, depth, 256, camera_surface);


		//suggestParameter[9] = 0;
		//suggestParameter[10] = 0;
		//suggestParameter[11] = PI / 2;
		//suggestParameter[12] = 0;
		//suggestParameter[13] = PI / 2;
		//suggestParameter[14] = PI / 2;
		//suggestParameter[15] = PI / 2;
		//suggestParameter[16] = 0;
		//suggestParameter[17] = PI / 2;
		//suggestParameter[18] = PI / 2;
		//suggestParameter[19] = PI / 2;
		//suggestParameter[20] = 0;
		//suggestParameter[21] = PI / 2;
		//suggestParameter[22] = PI / 2;
		//suggestParameter[23] = PI / 2;
		//suggestParameter[24] = 0;
		//suggestParameter[25] = PI / 2;
		//suggestParameter[26] = PI / 2;

		//trackParameter[9] = 0;
		//trackParameter[10] = 0;
		//trackParameter[11] = PI / 2;
		//trackParameter[12] = 0;
		//trackParameter[13] = PI / 2;
		//trackParameter[14] = PI / 2;
		//trackParameter[15] = PI / 2;
		//trackParameter[16] = 0;
		//trackParameter[17] = PI / 2;
		//trackParameter[18] = PI / 2;
		//trackParameter[19] = PI / 2;
		//trackParameter[20] = 0;
		//trackParameter[21] = PI / 2;
		//trackParameter[22] = PI / 2;
		//trackParameter[23] = PI / 2;
		//trackParameter[24] = 0;
		//trackParameter[25] = PI / 2;
		//trackParameter[26] = PI / 2;
		//if(useSuggestion)
		//DBOUT("\n detectePalm");
	

		trackParameter = optimized(trackParameter, suggestParameter, rh_d, depthMap, dist, camera_surface, useSuggestion);

		handEncoding(trackParameter, output);

		CameraSpacePoint* spacePoint = new CameraSpacePoint[48];
		DepthSpacePoint* depthPoints = new DepthSpacePoint[48];
		for (int i = 0; i < 16; i++)
		{
			worldtoCameraSpace(spacePoint[i], output[i].x, output[i].y, output[i].z);
			m_pCoordinateMapper->MapCameraPointToDepthSpace(spacePoint[i], &depthPoints[i]);
			circle(color, cv::Point(depthPoints[i].X, depthPoints[i].Y), 1, cv::Scalar(255, 255, 255), -1);
		}
		for (int i = 16; i < 22; i++)
		{
			worldtoCameraSpace(spacePoint[i], output[i].x, output[i].y, output[i].z);
			m_pCoordinateMapper->MapCameraPointToDepthSpace(spacePoint[i], &depthPoints[i]);
			circle(color, cv::Point(depthPoints[i].X, depthPoints[i].Y), 1, cv::Scalar(255, 0, 255), -1);
		}
		for (int i = 22; i < 28; i++)
		{
			worldtoCameraSpace(spacePoint[i], output[i].x, output[i].y, output[i].z);
			m_pCoordinateMapper->MapCameraPointToDepthSpace(spacePoint[i], &depthPoints[i]);
			circle(color, cv::Point(depthPoints[i].X, depthPoints[i].Y), 1, cv::Scalar(255, 0, 0), -1);
		}
		for (int i = 28; i < 34; i++)
		{
			worldtoCameraSpace(spacePoint[i], output[i].x, output[i].y, output[i].z);
			m_pCoordinateMapper->MapCameraPointToDepthSpace(spacePoint[i], &depthPoints[i]);
			circle(color, cv::Point(depthPoints[i].X, depthPoints[i].Y), 1, cv::Scalar(0, 255, 0), -1);
		}
		for (int i = 34; i < 40; i++)
		{
			worldtoCameraSpace(spacePoint[i], output[i].x, output[i].y, output[i].z);
			m_pCoordinateMapper->MapCameraPointToDepthSpace(spacePoint[i], &depthPoints[i]);
			circle(color, cv::Point(depthPoints[i].X, depthPoints[i].Y), 1, cv::Scalar(255, 255, 0), -1);
		}
		for (int i = 40; i < 48; i++)
		{
			worldtoCameraSpace(spacePoint[i], output[i].x, output[i].y, output[i].z);
			m_pCoordinateMapper->MapCameraPointToDepthSpace(spacePoint[i], &depthPoints[i]);
			circle(color, cv::Point(depthPoints[i].X, depthPoints[i].Y), 1, cv::Scalar(0, 255, 255), -1);
		}

		imshow("Debug Img", color);
	}




	return output;



}


void Tracker::setKinectParameter(int depthWidth, int depthHeight, int colorWidth, int colorHeight,
	USHORT minDistance, USHORT maxDistance, UINT16* depthBuffer, RGBQUAD* colorBuffer, ICoordinateMapper* coodinateMapper)
{
	nDepthWidth = depthWidth;
	nDepthHeight = depthHeight;
	nColorWidth = colorWidth;
	nColorHeight = colorHeight;

	nMinDistance = minDistance;
	nMaxDistance = maxDistance;

	pDepthBuffer = depthBuffer;
	pColorBuffer = colorBuffer;
	m_pCoordinateMapper = coodinateMapper;
	coordinateMapping(depthBuffer);
}

void Tracker::wristDetection()
{
	Mat color_img(nColorHeight, nColorWidth, CV_8UC4, reinterpret_cast<void*>(pColorBuffer));
	Mat  color_resize, color_hsv;
	resize(color_img, color_resize, Size(160, 90));
	cvtColor(color_resize, color_hsv, CV_BGR2HSV);
	inRange(color_hsv, Scalar(170, 180, 180), Scalar(179, 255, 255), img_WristMask);

	int dilateSize = 5;
	Mat element = getStructuringElement(MORPH_ELLIPSE,
		Size(2 * dilateSize + 1, 2 * dilateSize + 1),
		Point(dilateSize, dilateSize));
	dilate(img_WristMask, img_WristMask, element);

	imshow("wrist mask", img_WristMask);
}

bool Tracker::headTrack(Point3f &p3d)
{
	bool detectedHead = false;
	Mat depthMap(nDepthHeight, nDepthWidth, CV_16UC1, reinterpret_cast<void*>(pDepthBuffer));
	Mat head_depth = depthMap.clone();
	float topValue = 0;
	float position_x = 9999; 
	float position_y = 9999;
	float position_z = 9999;
	for (int j = 0; j < nDepthHeight; j++)
	{
		ushort* p = head_depth.ptr<ushort>(j);
		for (int i = 0; i < nDepthWidth; i++)
		{
			// convert from camera space - > world space
			float x, y, z;
			cameraToWorldSpace(depthToCamera_points[j*nDepthWidth + i], &x, &y, &z);
			//TODO Make it work with different setup
			if (y < 0.0 || z > -0.9 || z < -1.2f)
			{
				p[i] = 0;
			}
			else
			{
				if (y > topValue && !isinf(y))
				{
					position_x = x;
					position_y = y;
					position_z = z;
					topValue = y;
				}
			}
		}
	}

	if (position_y > 0 && position_y < 2.0 && position_z > -1.2f && position_z < -0.9)
	{
		detectedHead = true;
		p3d.x = position_x;
		p3d.y = position_y - 0.5;
		p3d.z = position_z;

		CameraSpacePoint cam_headPosition;
		DepthSpacePoint depth_headPosition;
		worldtoCameraSpace(cam_headPosition, position_x, position_y - 0.15, position_z);
		m_pCoordinateMapper->MapCameraPointToDepthSpace(cam_headPosition, &depth_headPosition);


		Mat head_img(nDepthHeight, nDepthWidth, CV_8UC1);
		head_depth.convertTo(head_img, CV_8UC1, 255.0 / (nMaxDistance - nMinDistance));

		circle(head_img, Point(depth_headPosition.X, depth_headPosition.Y), 2, Scalar(255));
		imshow("head img", head_img);

	}



	//float threshold = 50;
	//float max_area = 0;
	//int max_id;
	//vector<vector<Point>> contours;
	//vector<Vec4i> hierarchy;
	//vector<int> small_blobs;
	//float contour_area;
	//Mat temp_img;
	//head_img.copyTo(temp_img);

	//cv::findContours(temp_img, contours, hierarchy, CV_RETR_CCOMP,
	//	CV_CHAIN_APPROX_SIMPLE);

	//// Find indices of contours whose area is less than `threshold`
	//if (!contours.empty()) {
	//	for (size_t i = 0; i < contours.size(); ++i) {
	//		contour_area = contourArea(contours[i]);
	//		if (contour_area < threshold)
	//			small_blobs.push_back(i);

	//		if (contour_area > max_area)
	//		{
	//			max_area = contour_area;
	//			max_id = i;
	//		}
	//	}
	//}


	//if (max_area > threshold)
	//{
	//	detectedHead = true;
	//	Moments mu = moments(contours[max_id], false);
	//	Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
	//	//vector<vector<Point> > contours_poly(1);
	//	//approxPolyDP(Mat(contours[max_id]), contours_poly[0], 3, true);
	//	//Rect rec = boundingRect(contours_poly[0]);
	//	//int x = rec.x + rec.width / 2;
	//	//int y = rec.y;
	//	int index = (int)mc.y*nDepthWidth + mc.x;
	//	circle(head_img, mc, 3, Scalar(255), 1);
	//	cameraToWorldSpace(depthToCamera_points[index], &p3d.x, &p3d.y, &p3d.z);
	//}


	return detectedHead;

}

void Tracker::coordinateMapping(UINT16* pDepthBuffer)
{
	if (depthToCamera_points == NULL)
	depthToCamera_points = new CameraSpacePoint[nDepthHeight*nDepthWidth];

	if (depthToColor_points == NULL)
		depthToColor_points = new ColorSpacePoint[nColorHeight* nColorWidth];

	m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthHeight*nDepthWidth, pDepthBuffer, nDepthHeight*nDepthWidth, depthToCamera_points);
	m_pCoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth*nDepthHeight, pDepthBuffer, nDepthHeight*nDepthWidth, depthToColor_points);
}

void Tracker::filterInteractiveArea()
{
	detectedHandRight = false;
	Mat _depthMap(nDepthHeight, nDepthWidth, CV_16UC1, reinterpret_cast<void*>(pDepthBuffer));
	depthMap = _depthMap.clone();
	for (int j = 0; j < nDepthHeight; j++)
	{
		ushort* p = depthMap.ptr<ushort>(j);
		for (int i = 0; i < nDepthWidth; i++)
		{
			// convert from camera space - > world space
			float x, y, z;
			cameraToWorldSpace(depthToCamera_points[j*nDepthWidth + i], &x, &y, &z);

			//TODO Make it work with different setup
			if (x < -0.3 || x > 0.3 || y < 0.0 || y > 0.5 || z < -0.65 || z > 0.01)
			{
				p[i] = 0;
			}
			ColorSpacePoint colorPoint = depthToColor_points[j*nDepthWidth + i];
			int colorX = (int)(floor(colorPoint.X + 0.5)) / 12;
			int colorY = (int)(floor(colorPoint.Y + 0.5)) / 12;
			if ((colorX >= 0) && (colorX < 160) && (colorY >= 0) && (colorY < 90))
			{
				if (img_WristMask.at<uchar>(colorY, colorX) != 0)
					p[i] = 0;
			}
		}
	}

	depthMap.convertTo(depthMap8UC, CV_8UC1, 255.0 / (nMaxDistance - nMinDistance));

	float threshold = 50;
	float max_area = 0;
	int max_id = -1;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	vector<int> small_blobs;
	float contour_area;
	Mat temp_img;
	depthMap8UC.copyTo(temp_img);

	cv::findContours(temp_img, contours, hierarchy, CV_RETR_CCOMP,
		CV_CHAIN_APPROX_SIMPLE);

	// Find indices of contours whose area is less than `threshold`
	if (!contours.empty()) {
		for (size_t i = 0; i < contours.size(); ++i) {
			contour_area = contourArea(contours[i]);
			if (contour_area < threshold)
				small_blobs.push_back(i);

			if (contour_area > max_area)
			{
				max_area = contour_area;
				max_id = i;
			}
		}
	}

	// fill-in all small contours with zeros
	for (size_t i = 0; i < contours.size(); ++i) {
		if (i != max_id)
		{
			drawContours(depthMap8UC, contours, i, cv::Scalar(0), CV_FILLED, 8);
			drawContours(depthMap, contours, i, cv::Scalar(0), CV_FILLED, 8);

		}
	}
	Point2f mc;
	if (max_area > threshold && max_id != -1)
	{

		//Find Hand Center
		Moments mu = moments(contours[max_id], false);
		Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

		if (mc.x < nDepthWidth - 75 && mc.x> 75 && mc.y < nDepthHeight - 75 && mc.y > 75)
		{
			rh_d.x = mc.x;
			rh_d.y = mc.y;
			detectedHandRight = true;

			int centerHand_index = (int)mc.y * nDepthWidth + mc.x;
			float center_y,center_x, center_z;
			cameraToWorldSpace(depthToCamera_points[centerHand_index], &center_x, &center_y, &center_z);


			for (int j = mc.y - 75; j < mc.y + 75; j++)
			{
				ushort* p = depthMap.ptr<ushort>(j);
				uchar* q = depthMap8UC.ptr<uchar>(j);
				for (int i = mc.x - 75; i < mc.x + 75; i++)
				{
					// convert from camera space - > world space
					float y, x, z;
					cameraToWorldSpace(depthToCamera_points[j*nDepthWidth + i], &x, &y, &z);
					//TODO Make it work with different setup
					if (center_y - 0.08 > y)
					{
						p[i] = 0;
						q[i] = 0;
					}
				}
			}
		}
	}
}

vector<Point> Tracker::findExtremePoint(Mat img_blackwhite)
{

	vector<Point> extremePointList;

	//------------ Initialized Djikstra adjacent matrix graph -------------------
	for (int i = 0; i < img_blackwhite.rows; ++i)
	{
		for (int j = 0; j < img_blackwhite.cols; ++j)
		{
			a[i*img_blackwhite.rows + j].clear();
			vis[i*img_blackwhite.rows + j] = false;
			dis[i*img_blackwhite.rows + j] = INF;
		}
	}
	for (int i = 0; i < img_blackwhite.rows; ++i)
	{
		for (int j = 0; j < img_blackwhite.cols; ++j)
		{
			if (img_blackwhite.at<uchar>(i, j) > 0 && i > 1 && i < 149 && j > 1 && j < 149)
			{
				if (img_blackwhite.at<uchar>(i + 1, j) > 0)
				{
					a[i*img_blackwhite.cols + j].push_back(make_pair((i + 1)*img_blackwhite.cols + j, 1));
					a[(i + 1)*img_blackwhite.cols + j].push_back(make_pair(i*img_blackwhite.cols + j, 1));
				}
				if (img_blackwhite.at<uchar>(i, j + 1) > 0)
				{
					a[i*img_blackwhite.cols + j].push_back(make_pair(i*img_blackwhite.cols + j + 1, 1));
					a[i*img_blackwhite.cols + j + 1].push_back(make_pair(i*img_blackwhite.cols + j, 1));
				}
				if (img_blackwhite.at<uchar>(i + 1, j + 1) > 0)
				{
					a[i*img_blackwhite.cols + j].push_back(make_pair((i + 1)*img_blackwhite.cols + j + 1, 1.414));
					a[(i + 1)*img_blackwhite.cols + j + 1].push_back(make_pair(i*img_blackwhite.cols + j, 1.414));
				}
				if (img_blackwhite.at<uchar>(i + 1, j - 1) > 0)
				{
					a[i*img_blackwhite.cols + j].push_back(make_pair((i + 1)*img_blackwhite.cols + j - 1, 1.414));
					a[(i + 1)*img_blackwhite.cols + j - 1].push_back(make_pair(i*img_blackwhite.cols + j, 1.414));
				}
			}
		}
	}

	int source = 75 * img_blackwhite.cols + 75;
	dijkstra(source, 22500);
	for (int k = 0; k < 5; k++)
	{
		float extremePoint_value = 0;
		int extremePointIndex = 0;
		int extremePoint_x = 0;
		int extremePoint_y = 0;
		for (int i = 0; i < 22500; i++)
		{
			if (dis[i] > extremePoint_value && dis[i] != INF)
			{
				extremePoint_value = dis[i];
				extremePointIndex = i;
			}
			vis[i] = false;
		}

		extremePoint_x = extremePointIndex % 150;
		extremePoint_y = (int)(extremePointIndex / 150);
		extremePointList.push_back(cv::Point(extremePoint_x, extremePoint_y));
		dis[extremePoint_y * 150 + extremePoint_x] = 0;
		dijkstra(extremePoint_y * 150 + extremePoint_x, 22500);
	}

	float extremePoint_value = 0;
	int extremePoint_x = 0;
	int extremePointIndex = 0;
	RNG rng(0xFFFFFFFF);
	int extremePoint_y = 0;
	for (int i = 0; i < 22500; i++)
	{
		if (dis[i] > extremePoint_value && dis[i] != INF)
		{
			extremePoint_value = dis[i];
			extremePointIndex = i;
		}
		vis[i] = false;
	}
	extremePoint_x = extremePointIndex % 150;
	extremePoint_y = (int)(extremePointIndex / 150);
	extremePointList.push_back(cv::Point(extremePoint_x, extremePoint_y));

	return extremePointList;

}

void Tracker::dijkstra(int source, int n) //Algorithm for SSSP
{
	//AllocConsole();
	//freopen("CONIN$", "r", stdin);
	//freopen("CONOUT$", "w", stdout);
	//freopen("CONOUT$", "w", stderr);

	priority_queue<pair<int, float>, vector<pair<int, float> >, comparator> pq; //Priority queue to store vertex,weight pairs
	pq.push(make_pair(source, dis[source] = 0)); //Pushing the source with distance from itself as 0
	while (!pq.empty())
	{
		pair<int, float> curr = pq.top(); //Current vertex. The shortest distance for this has been found
		pq.pop();
		int cv = curr.first, cw = curr.second;
		vis[cv] = true; //This is the final shortest distance
		for (int i = 0; i < a[cv].size(); i++) //Iterating through all adjacent vertices
			if (!vis[a[cv][i].first]) //If this vertex is not visited, that means
				if (a[cv][i].second + cw < dis[a[cv][i].first]) //If current parent node distance+distance from there to this node is shorted than the initial distace set to this node, update it
				{
			pq.push(make_pair(a[cv][i].first, (dis[a[cv][i].first] = a[cv][i].second + cw))); //Set the new distance and add to priority queue
			//	pred[a[cv][i].first] = cv;
				}
	}
	//cout << "Source is: " << source << ". The shortest distance to every other vertex from here is: \n";
	//for (int i = 1; i <= n; i++)//Printing final shortest distances from source
	//{
	//	cout << "Vertex: " << i << " , Distance: ";
	//	if (dis[i] != INF)
	//		cout << dis[i] << endl;
	//	else
	//		cout << "-1\n";
	//}
}

void Tracker::findXYFingerSegment(vector<Point> extremePointList, Mat &img_blackwhite, ushort hand_depth, Point2f rh_d)
{
	fingertip_list.resize(6);
	xyFingerSegmented = Mat::zeros(150, 150, CV_8U);

	for (int i = 0; i < extremePointList.size(); i++)
	{
		int count = 100;
		fingerSegmented[i] = Mat::zeros(150, 150, CV_8U);
		valid_finger[i] = false;
		if (breadfirst(cv::Point(extremePointList[i].x, extremePointList[i].y), 255, img_blackwhite, fingerSegmented[i], hand_depth))
		{
			//circle(color, Point(extremePointList[i].x + rh_d.X - 75, extremePointList[i].y + rh_d.Y - 75), 3, randomColor(rng), 3);
			fingertip_list[i] = (Point(extremePointList[i].x + rh_d.x - 75, extremePointList[i].y + rh_d.y - 75));
			valid_finger[i] = true;
			xyFingerSegmented += fingerSegmented[i];
		}
	}

}

void Tracker::findZFingerSegment(Mat img_depthHand, Point2f rh_d)
{
	for (int j = 0; j < 150; j++)
	{
		for (int i = 0; i < 150; i++)
		{
			if (localMinimum(i, j, img_depthHand) && xyFingerSegmented.at<uchar>(j, i) == 0)
				if (localAngle(i, j, img_depthHand))
				{
				int k = 0;
				while (valid_finger[k] && k < 5)
					k++;

				fingerSegmented[k] = Mat::zeros(150, 150, CV_8U);
				ushort depth = img_depthHand.at<ushort>(j, i);
				//circle(color, Point(i + rh_d.X - 75, j + rh_d.Y - 75), 3, randomColor(rng), 3);
				floodfill4(i, j, i, j, 0, 255, depth, fingerSegmented[k], img_depthHand);
				valid_finger[k] = true;
				fingertip_list[k] = (Point(i + rh_d.x - 75, j + rh_d.y - 75));
				xyFingerSegmented += fingerSegmented[k];
				}
		}
	}

}

bool Tracker::breadfirst(cv::Point root, uchar color, const Mat& img, Mat& out, ushort handCenter_depth)
{
	std::queue<Point> Q;
	std::queue<Point> another_Q;
	Q.push(root);

	float convert_pixel_to_mm = handCenter_depth*0.708 / 256.0f;
	cv::Point next;

	float length_limit = 8 * convert_pixel_to_mm;
	float width_limit = 4 * convert_pixel_to_mm;
	int length = 0;
	int width = 0;

	while (width < (int)width_limit && length < (int)length_limit)
	{
		width = 0;
		while (!Q.empty())
		{
			cv::Point current = Q.front();
			float maximum = 0.0;
			Q.pop();
			int x = current.x;
			int y = current.y;
			if (pixelFill(current.x + 1, current.y, color, img, out))
			{
				width++;
				another_Q.push(Point(current.x + 1, current.y));
			}
			if (pixelFill(current.x - 1, current.y, color, img, out))
			{
				width++;
				another_Q.push(Point(current.x - 1, current.y));
			}
			if (pixelFill(current.x, current.y + 1, color, img, out))
			{
				width++;
				another_Q.push(Point(current.x, current.y + 1));
			}
			if (pixelFill(current.x, current.y - 1, color, img, out))
			{
				width++;
				another_Q.push(Point(current.x, current.y - 1));
			}
		}
		length++;
		Q.swap(another_Q);
	}
	if (length > (width * 1.5))
	{
		return true;
	}
	else
		return false;
}

bool Tracker::pixelFill(int point_x, int point_y, uchar color, const Mat& img, Mat& out)
{
	if (img.empty() || out.empty() || point_x <= 0 || point_x >= 150 || point_y <= 0 || point_y >= 150)
		return false;

	if (img.at<uchar>(point_y, point_x) == color && out.at<uchar>(point_y, point_x) != color)
	{
		out.at<uchar>(point_y, point_x) = color;
		return true;
	}
	else
	{
		return false;
	}
}

bool Tracker::localMinimum(int point_x, int point_y, Mat& img)
{
	if (point_x <= 1 || point_x >= 148 || point_y <= 1 || point_y >= 148 || img.empty())
	{
		return false;
	}

	if (img.at<ushort>(point_y, point_x) > 0 &&
		img.at<ushort>(point_y, point_x) <= img.at<ushort>(point_y + 1, point_x) &&
		img.at<ushort>(point_y, point_x) <= img.at<ushort>(point_y - 1, point_x) &&
		img.at<ushort>(point_y, point_x) <= img.at<ushort>(point_y, point_x + 1) &&
		img.at<ushort>(point_y, point_x) <= img.at<ushort>(point_y, point_x - 1) &&
		img.at<ushort>(point_y, point_x) <= img.at<ushort>(point_y + 1, point_x + 1) &&
		img.at<ushort>(point_y, point_x) <= img.at<ushort>(point_y - 1, point_x - 1) &&
		img.at<ushort>(point_y, point_x) <= img.at<ushort>(point_y - 1, point_x + 1) &&
		img.at<ushort>(point_y, point_x) <= img.at<ushort>(point_y + 1, point_x - 1))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool Tracker::localAngle(int point_x, int point_y, Mat& img)
{
	if (point_x <= 5 || point_x >= 144 || point_y <= 5 || point_y >= 144 || img.empty())
	{
		return false;
	}

	int bin_count = 0;

	if (img.at<ushort>(point_y, point_x) + 15 >= img.at<ushort>(point_y + 5, point_x) && img.at<ushort>(point_y + 5, point_x) != 0) bin_count++;
	if (img.at<ushort>(point_y, point_x) + 15 >= img.at<ushort>(point_y - 5, point_x) && img.at<ushort>(point_y - 5, point_x) != 0) bin_count++;
	if (img.at<ushort>(point_y, point_x) + 15 >= img.at<ushort>(point_y, point_x + 5) && img.at<ushort>(point_y, point_x + 5) != 0) bin_count++;
	if (img.at<ushort>(point_y, point_x) + 15 >= img.at<ushort>(point_y, point_x - 5) && img.at<ushort>(point_y, point_x - 5) != 0) bin_count++;
	if (img.at<ushort>(point_y, point_x) + 15 >= img.at<ushort>(point_y + 2, point_x + 2) && img.at<ushort>(point_y + 2, point_x + 2) != 0) bin_count++;
	if (img.at<ushort>(point_y, point_x) + 15 >= img.at<ushort>(point_y + 2, point_x - 2) && img.at<ushort>(point_y + 2, point_x - 2) != 0) bin_count++;
	if (img.at<ushort>(point_y, point_x) + 15 >= img.at<ushort>(point_y - 2, point_x + 2) && img.at<ushort>(point_y - 2, point_x + 2) != 0) bin_count++;
	if (img.at<ushort>(point_y, point_x) + 15 >= img.at<ushort>(point_y - 2, point_x - 2) && img.at<ushort>(point_y - 2, point_x - 2) != 0) bin_count++;

	if (bin_count < 2)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Tracker::floodfill4(int x, int y, int center_x, int center_y, uchar oldColor, uchar newColor, ushort depth, Mat& img, Mat& depthMap)
{
	std::queue<int> Q_x;
	std::queue<int> Q_y;

	Mat processed = Mat::zeros(150, 150, CV_8U);
	Q_x.push(x);
	Q_y.push(y);

	while (!Q_x.empty())
	{
		int n_x = Q_x.front();
		int n_y = Q_y.front();
		uchar* p = img.data;
		processed.at<uchar>(n_y, n_x) = 255;

		if (n_x >= 1 && n_x < 149 && n_y >= 1 && n_y < 149 && n_x > center_x - 10 && n_x < center_x + 10 && n_y > center_y - 10 && n_y < center_y + 10 &&
			p[n_y * 150 + n_x] == oldColor && p[n_y * 150 + n_x] != newColor && depthMap.at<ushort>(n_y, n_x) != 0 && depthMap.at<ushort>(n_y, n_x) - depth < 50)
		{
			p[n_y * 150 + n_x] = newColor;//set color before starting recursion

			if (processed.at<uchar>(n_y + 1, n_x) == 0) { Q_x.push(n_x); Q_y.push(n_y + 1); }
			if (processed.at<uchar>(n_y - 1, n_x) == 0) { Q_x.push(n_x); Q_y.push(n_y - 1); }
			if (processed.at<uchar>(n_y, n_x + 1) == 0) { Q_x.push(n_x + 1); Q_y.push(n_y); }
			if (processed.at<uchar>(n_y, n_x - 1) == 0) { Q_x.push(n_x - 1); Q_y.push(n_y); }
		}

		Q_x.pop();
		Q_y.pop();
	}
}

void Tracker::pcaPalm(Mat img_depthHand, Point2f rh_d, Mat color)
{
	globalTranslate = Mat::eye(4, 4, CV_32F);
	globalRotate = Mat::eye(4, 4, CV_32F);
	globalRotateReverse = Mat::eye(4, 4, CV_32F);
	detectedPalm = false;

	//filter out only palm
	vector<Point3f> pts;
	for (int j = rh_d.y - 75, m = 0; j < rh_d.y + 75, m < 150; j++, m++)
	{
		for (int i = rh_d.x - 75, n = 0; j < rh_d.x + 75, n < 150; i++, n++)
		{
			//add palm segment to the PCA, convert camera space to world coordinate first
			if (xyFingerSegmented.at<uchar>(m, n) != 255 && img_depthHand.at<ushort>(m, n) != 0)
			{
				int index = j*nDepthWidth + i;
				CameraSpacePoint p = depthToCamera_points[index];
				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
				{
					float x, y, z;

					cameraToWorldSpace(p, &x, &y, &z);
					pts.push_back(Point3f(x, y, z));
				}
			}
		}
	}

	int sz = static_cast<int>(pts.size());
	if (sz > 0)
	{
		Mat data_pts = Mat(sz, 3, CV_32FC1);
		for (int i = 0; i < data_pts.rows; ++i)
		{
			data_pts.at<float>(i, 0) = pts[i].x;
			data_pts.at<float>(i, 1) = pts[i].y;
			data_pts.at<float>(i, 2) = pts[i].z;
		}
		PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

		Point3d cntr = Point3d(pca_analysis.mean.at<float>(0, 0),
			pca_analysis.mean.at<float>(0, 1), pca_analysis.mean.at<float>(0, 2));

		palmCenter = cntr;

		suggestParameter[0] = cntr.x;
		suggestParameter[1] = cntr.y;
		suggestParameter[2] = cntr.z;

		int eigen_vector_size = pca_analysis.eigenvectors.rows;
		//Store the eigenvalues and eigenvectors
		vector<Point3d> eigen_vecs(3);
		vector<float> eigen_val(3);

		if (eigen_vector_size >= 2)
		{
			//normalize vector
			float norm[2];
			norm[0] = sqrt(pca_analysis.eigenvectors.at<float>(0, 0)* pca_analysis.eigenvectors.at<float>(0, 0) +
				pca_analysis.eigenvectors.at<float>(0, 1) * pca_analysis.eigenvectors.at<float>(0, 1) +
				pca_analysis.eigenvectors.at<float>(0, 2) * pca_analysis.eigenvectors.at<float>(0, 2));

			norm[1] = sqrt(pca_analysis.eigenvectors.at<float>(1, 0)* pca_analysis.eigenvectors.at<float>(1, 0) +
				pca_analysis.eigenvectors.at<float>(1, 1) * pca_analysis.eigenvectors.at<float>(1, 1) +
				pca_analysis.eigenvectors.at<float>(1, 2) * pca_analysis.eigenvectors.at<float>(1, 2));

			for (int i = 0; i < eigen_vector_size; i++)
			{
				eigen_vecs[i] = Point3d(pca_analysis.eigenvectors.at<float>(i, 0) / norm[i],
					pca_analysis.eigenvectors.at<float>(i, 1) / norm[i], pca_analysis.eigenvectors.at<float>(i, 2) / norm[i]);
				eigen_val[i] = pca_analysis.eigenvalues.at<float>(i, 0);
			}

			palmFaceVector = eigen_vecs[1].cross(eigen_vecs[0]);

			globalRotate.at<float>(0, 0) = eigen_vecs[1].x;
			globalRotate.at<float>(0, 1) = eigen_vecs[1].y;
			globalRotate.at<float>(0, 2) = eigen_vecs[1].z;
			globalRotate.at<float>(0, 3) = 0;

			globalRotate.at<float>(1, 0) = eigen_vecs[0].x;
			globalRotate.at<float>(1, 1) = eigen_vecs[0].y;
			globalRotate.at<float>(1, 2) = eigen_vecs[0].z;
			globalRotate.at<float>(1, 3) = 0;

			globalRotate.at<float>(2, 0) = palmFaceVector.x;
			globalRotate.at<float>(2, 1) = palmFaceVector.y;
			globalRotate.at<float>(2, 2) = palmFaceVector.z;
			globalRotate.at<float>(2, 3) = 0;

			globalRotate = globalRotate.inv();

			globalRotateReverse.at<float>(0, 0) = -eigen_vecs[1].x;
			globalRotateReverse.at<float>(0, 1) = -eigen_vecs[1].y;
			globalRotateReverse.at<float>(0, 2) = -eigen_vecs[1].z;
			globalRotateReverse.at<float>(0, 3) = 0;

			globalRotateReverse.at<float>(1, 0) = eigen_vecs[0].x;
			globalRotateReverse.at<float>(1, 1) = eigen_vecs[0].y;
			globalRotateReverse.at<float>(1, 2) = eigen_vecs[0].z;
			globalRotateReverse.at<float>(1, 3) = 0;

			globalRotateReverse.at<float>(2, 0) = -palmFaceVector.x;
			globalRotateReverse.at<float>(2, 1) = -palmFaceVector.y;
			globalRotateReverse.at<float>(2, 2) = -palmFaceVector.z;
			globalRotateReverse.at<float>(2, 3) = 0;

			globalRotateReverse = globalRotateReverse.inv();

			detectedPalm = true;

			globalTranslate.at<float>(0, 3) = cntr.x;
			globalTranslate.at<float>(1, 3) = cntr.y;
			globalTranslate.at<float>(2, 3) = cntr.z;

			CameraSpacePoint cntr_cam;
			DepthSpacePoint cntr_depth;

			worldtoCameraSpace(cntr_cam, cntr.x, cntr.y, cntr.z);
			m_pCoordinateMapper->MapCameraPointToDepthSpace(cntr_cam, &cntr_depth);
			circle(color, cv::Point(cntr_depth.X, cntr_depth.Y), 5, cv::Scalar(255, 0, 255), -1);

			for (int i = 0; i < eigen_vector_size; i++)
			{
				Point3d p1 = cntr + 0.1*Point3d(eigen_vecs[i].x , eigen_vecs[i].y, eigen_vecs[i].z);
				DepthSpacePoint p1_depth;
				worldtoCameraSpace(cntr_cam, p1.x, p1.y, p1.z);
				m_pCoordinateMapper->MapCameraPointToDepthSpace(cntr_cam, &p1_depth);
				drawAxis(color, cv::Point(cntr_depth.X, cntr_depth.Y), cv::Point(p1_depth.X, p1_depth.Y), Scalar(255 * i, 255, 255 * (i / 2)), 1);
			}
		}
	}
}

void Tracker::pcaFinger(Point2f rh_d, Mat color)
{
	palmEnergy = 0;
	//for every finger segment found
	for (int k = 0; k < 6; k++)
	{
		if (valid_finger[k] && detectedPalm)
		{
			vector<Point3f> pts_finger;
			for (int j = rh_d.y - 75, m = 0; j < rh_d.y + 75, m < 150; j++, m++)
			{
				for (int i = rh_d.x - 75, n = 0; j < rh_d.x + 75, n < 150; i++, n++)
				{
					//add segment to the PCA, convert camera space to world coordinate first
					if (fingerSegmented[k].at<uchar>(m, n) > 0)
					{
						int index = j*nDepthWidth + i;
						CameraSpacePoint p = depthToCamera_points[index];
						if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
						{
							float x, y, z;
							cameraToWorldSpace(p, &x, &y, &z);
							pts_finger.push_back(Point3f(x, y, z));
						}
					}
				}
			}

			int sz = static_cast<int>(pts_finger.size());
			if (sz > 0)
			{
				Mat data_pts = Mat(sz, 3, CV_32FC1);
				for (int i = 0; i < data_pts.rows; ++i)
				{
					data_pts.at<float>(i, 0) = pts_finger[i].x;
					data_pts.at<float>(i, 1) = pts_finger[i].y;
					data_pts.at<float>(i, 2) = pts_finger[i].z;
				}

				PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

				Point3d cntr = Point3d(pca_analysis.mean.at<float>(0, 0),
					pca_analysis.mean.at<float>(0, 1), pca_analysis.mean.at<float>(0, 2));

				Point3d direction = cntr - palmCenter;

				int eigen_vector_size = pca_analysis.eigenvectors.rows;
				//Store the eigenvalues and eigenvectors
				vector<Point3d> eigen_vecs(eigen_vector_size);
				vector<float> eigen_val(eigen_vector_size);

				Point3d max_eigen_vecs;
				float max_eigen_val;
				float max_value = -999999;

				for (int i = 0; i < eigen_vector_size; i++)
				{
					eigen_vecs[i] = Point3d(pca_analysis.eigenvectors.at<float>(i, 0),
						pca_analysis.eigenvectors.at<float>(i, 1), pca_analysis.eigenvectors.at<float>(i, 2));
					eigen_val[i] = pca_analysis.eigenvalues.at<float>(i, 0);
					float project_value = eigen_vecs[i].dot(direction);
					if (project_value > max_value)
					{
						max_eigen_vecs = eigen_vecs[i];
						max_value = project_value;
						max_eigen_val = eigen_val[i];
					}

					project_value = (-eigen_vecs[i]).dot(direction);
					if (project_value > max_value)
					{
						max_eigen_vecs = (-eigen_vecs[i]);
						max_value = project_value;
						max_eigen_val = eigen_val[i];
					}
				}

				float norm = sqrt(max_eigen_vecs.x * max_eigen_vecs.x + max_eigen_vecs.y * max_eigen_vecs.y + max_eigen_vecs.z * max_eigen_vecs.z);
				Point3d temp_direction = Point3d(max_eigen_vecs.x / norm, max_eigen_vecs.y / norm, max_eigen_vecs.z / norm);
				palmEnergy += temp_direction.dot(palmFaceVector);

				fingerDirection.push_back(temp_direction);
				int index = fingertip_list[k].y * nDepthWidth + fingertip_list[k].x;

				CameraSpacePoint p = depthToCamera_points[index];
				while (p.X == -std::numeric_limits<float>::infinity() || p.Y == -std::numeric_limits<float>::infinity())
				{
					index++;
					p = depthToCamera_points[index];
				}

				float x, y, z;
				cameraToWorldSpace(p, &x, &y, &z);
				fingerTipPosition.push_back(Point3d(x, y, z));

				//CameraSpacePoint fin_cam;
				//DepthSpacePoint fin_depth;
				//
				//m_pCoordinateMapper->MapCameraPointToDepthSpace(fin_cam, &fin_depth);
				//circle(color, cv::Point(fin_depth.X, fin_depth.Y), 3, cv::Scalar(0, 0, 255), 1);

				//CameraSpacePoint cntr_cam;
				//DepthSpacePoint cntr_depth;
				//cntr_cam.X = cntr.x;
				//cntr_cam.Y = 0.4226*(cntr.y - height) - 0.9063*cntr.z;
				//cntr_cam.Z = -0.9063*(cntr.y - height) - 0.4426*cntr.z;
				//m_pCoordinateMapper->MapCameraPointToDepthSpace(cntr_cam, &cntr_depth);
				//circle(color, cv::Point(cntr_depth.X, cntr_depth.Y), 2, cv::Scalar(255, 0, 255), -1);

				//Point3d p1 = cntr + 100.0* Point3d(max_eigen_vecs.x * max_eigen_val, max_eigen_vecs.y* max_eigen_val, max_eigen_vecs.z* max_eigen_val);
				//DepthSpacePoint p1_depth;
				//cntr_cam.X = p1.x;
				//cntr_cam.Y = 0.4226*(p1.y - height) - 0.9063*p1.z;
				//cntr_cam.Z = -0.9063*(p1.y - height) - 0.4426*p1.z;
				//m_pCoordinateMapper->MapCameraPointToDepthSpace(cntr_cam, &p1_depth);
				//drawAxis(color, cv::Point(cntr_depth.X, cntr_depth.Y), cv::Point(p1_depth.X, p1_depth.Y), Scalar(255, 255, 0), 1);
			}
		}
	}
}

void Tracker::predictFinger()
{
	
		Mat finger_base = (Mat_<float>(4, 5) << -particleRadius * 3, -particleRadius * 2, 0, particleRadius * 2, particleRadius * 4,
			-particleRadius*1.33, particleRadius * 4, particleRadius * 4, particleRadius * 3.5, particleRadius * 3,
			-particleRadius, 0, 0, 0, 0,
			1, 1, 1, 1, 1);

		float finger_scale[5] = { 0.072, 0.072, 0.084, 0.072, 0.06 };

		float q[4];

		float energy = 0;
		float energyReverse = 0;
		bool handReverse = false;
		bool lastHandOrientation = false;
		bool determined = false;
		Mat result(4, 5, CV_32F);
		Mat result_flip(4, 5, CV_32F);



		if (palmEnergy > 2.0 || palmEnergy < -2.0)
			determined = true;

		DBOUT(palmEnergy);

		if (palmEnergy > 2.0 && determined)
		{
			result = globalTranslate*globalRotate*finger_base;
			handReverse = true;
			lastHandOrientation = true;
			DBOUT("Determined normal");
		}
		else if (palmEnergy < -2.0 && determined)
		{
			result = globalTranslate*globalRotateReverse*finger_base;
			handReverse = false;
			lastHandOrientation = false;
			DBOUT("Determined reverse");
		}
		else
		{
			result = globalTranslate*globalRotate*finger_base;
			result_flip = globalTranslate*globalRotateReverse*finger_base;
		}

		vector<int> finger_index(5);
		vector<int> finger_index_flip(5);

		if (determined)
		{
			int r[5][5] = { { 0 } };
			int maximum = 0;
			for (int j = 0; j < fingerDirection.size() && j < 5; j++)
			{
				int position = -1;
				for (int i = 0; i < result.cols; i++)
				{
					//--Propagate fingertip posistion from hand orientation and finger direction--
					Point3d finger_forward = Point3d(result.at<float>(0, i) + finger_scale[i] * fingerDirection[j].x,
						result.at<float>(1, i) + finger_scale[i] * fingerDirection[j].y,
						result.at<float>(2, i) + finger_scale[i] * fingerDirection[j].z);
					//--Compare with the actual detected fingertip position--
					float distance = (fingerTipPosition[j].x - finger_forward.x)*(fingerTipPosition[j].x - finger_forward.x) +
						(fingerTipPosition[j].y - finger_forward.y)*(fingerTipPosition[j].y - finger_forward.y) +
						(fingerTipPosition[j].z - finger_forward.z)*(fingerTipPosition[j].z - finger_forward.z);

					r[i][j] = int(distance * 10000);
					if (int(distance * 10000) > maximum)
					{
						maximum = (int)(distance * 10000);
					}
				}
			}

			//--Pad undetected fingertip for Hungarian algorithm--
			for (int j = fingerDirection.size(); j < 5; j++)
			{
				for (int i = 0; i < result.cols; i++)
				{
					r[i][j] = maximum;
				}
			}

			//--Perform Hungarian algorithm--
			hungarian_t prob;
			//cout << "Hungarian problem 0" << endl;
			hungarian_init(&prob, (int*)r, 5, 5, HUNGARIAN_MIN);
			//hungarian_print_rating(&prob);
			hungarian_solve(&prob);

			//hungarian_print_assignment(&prob);
			//cout << "Hungarian solved" << endl;
			//-- assign result from the algorithm to the detected finger--

			for (int i = 0; i < 5; i++)
				for (int j = 0; j < fingerDirection.size() && j < 5; j++)
				{
				if (j == prob.a[i])
				{
					//cout << "finger insert" << i << " colum " << j << endl;
					finger_index[j] = i;
					energy += r[i][j];
				}
				}

			//--Clear Hungarian algorithm---
			hungarian_fini(&prob);

			for (int j = 0; j < fingerDirection.size() && j < 5; j++)
			{
				int i = finger_index[j];
				//cout << "Finger position " << i << endl;
				//Point3d finger_forward = Point3d(result.at<float>(0, i) + finger_scale[i] * fingerDirection[j].x,
				//	result.at<float>(1, i) + finger_scale[i] * fingerDirection[j].y,
				//	result.at<float>(2, i) + finger_scale[i] * fingerDirection[j].z);
				//Point3d finger_forward = Point3d(result.at<float>(0, i),
				//result.at<float>(1, i) ,
				//result.at<float>(2, i) );

				detected_finger[i] = true;
				Mat tempTransform;
				if (palmEnergy > 2.0)
					tempTransform = globalRotate;
				else
					tempTransform = globalRotateReverse;

				assignFingerTip(i, fingerDirection[j], tempTransform, suggestParameter);

				//CameraSpacePoint cntr_cam;
				//DepthSpacePoint cntr_depth;
				//cntr_cam.X = finger_forward.x;
				//cntr_cam.Y = 0.4226*(finger_forward.y - height) - 0.9063*finger_forward.z;
				//cntr_cam.Z = -0.9063*(finger_forward.y - height) - 0.4426*finger_forward.z;
				//m_pCoordinateMapper->MapCameraPointToDepthSpace(cntr_cam, &cntr_depth);
				//circle(color, cv::Point(cntr_depth.X, cntr_depth.Y), 3, cv::Scalar(i * 51, 255, 255), -1);
			}

			if (palmEnergy > 2.0)
			{
				m[0][0] = globalRotate.at<float>(0, 0);
				m[0][1] = globalRotate.at<float>(0, 1);
				m[0][2] = globalRotate.at<float>(0, 2);

				m[1][0] = globalRotate.at<float>(1, 0);
				m[1][1] = globalRotate.at<float>(1, 1);
				m[1][2] = globalRotate.at<float>(1, 2);

				m[2][0] = globalRotate.at<float>(2, 0);
				m[2][1] = globalRotate.at<float>(2, 1);
				m[2][2] = globalRotate.at<float>(2, 2);
			}
			else
			{
				m[0][0] = globalRotateReverse.at<float>(0, 0);
				m[0][1] = globalRotateReverse.at<float>(0, 1);
				m[0][2] = globalRotateReverse.at<float>(0, 2);

				m[1][0] = globalRotateReverse.at<float>(1, 0);
				m[1][1] = globalRotateReverse.at<float>(1, 1);
				m[1][2] = globalRotateReverse.at<float>(1, 2);

				m[2][0] = globalRotateReverse.at<float>(2, 0);
				m[2][1] = globalRotateReverse.at<float>(2, 1);
				m[2][2] = globalRotateReverse.at<float>(2, 2);
			}
		}
		else
		{
			DBOUT(" UnDetermined");
			//-- if not determinded before hand have to test both hand orientation--
			int r[5][5] = { { 0 } };
			int maximum = 0;
			for (int j = 0; j < fingerDirection.size() && j < 5; j++)
			{
				int position = -1;
				for (int i = 0; i < result.cols; i++)
				{
					Point3d finger_forward = Point3d(result.at<float>(0, i) + finger_scale[i] * fingerDirection[j].x,
						result.at<float>(1, i) + finger_scale[i] * fingerDirection[j].y,
						result.at<float>(2, i) + finger_scale[i] * fingerDirection[j].z);
					float distance = (fingerTipPosition[j].x - finger_forward.x)*(fingerTipPosition[j].x - finger_forward.x) +
						(fingerTipPosition[j].y - finger_forward.y)*(fingerTipPosition[j].y - finger_forward.y) +
						(fingerTipPosition[j].z - finger_forward.z)*(fingerTipPosition[j].z - finger_forward.z);

					r[i][j] = int(distance * 10000);
					if (int(distance * 10000) > maximum)
					{
						maximum = (int)(distance * 10000);
					}
				}
			}

			for (int j = fingerDirection.size(); j < 5; j++)
			{
				for (int i = 0; i < result.cols; i++)
				{
					r[i][j] = maximum;
				}
			}

			hungarian_t prob;
			//cout << "Hungarian problem 1 " << endl;
			hungarian_init(&prob, (int*)r, 5, 5, HUNGARIAN_MIN);
			//hungarian_print_rating(&prob);
			hungarian_solve(&prob);
			//hungarian_print_assignment(&prob);
			//cout << "Hungarian solved " << endl;

			for (int i = 0; i < 5; i++)
				for (int j = 0; j < fingerDirection.size() && j < 5; j++)
				{
				if (j == prob.a[i])
				{
					//cout << "finger insert" << i << " row " << j << endl;
					finger_index[j] = i;
					energy += r[i][j];
				}
				}

			hungarian_fini(&prob);

			for (int j = 0; j < fingerDirection.size() && j < 5; j++)
			{
				float minimized = 999999999;
				int position = -1;
				for (int i = 0; i < result_flip.cols; i++)
				{
					Point3d finger_forward = Point3d(result_flip.at<float>(0, i) + finger_scale[i] * fingerDirection[j].x,
						result_flip.at<float>(1, i) + finger_scale[i] * fingerDirection[j].y,
						result_flip.at<float>(2, i) + finger_scale[i] * fingerDirection[j].z);
					float distance = (fingerTipPosition[j].x - finger_forward.x)*(fingerTipPosition[j].x - finger_forward.x) +
						(fingerTipPosition[j].y - finger_forward.y)*(fingerTipPosition[j].y - finger_forward.y) +
						(fingerTipPosition[j].z - finger_forward.z)*(fingerTipPosition[j].z - finger_forward.z);

					r[i][j] = int(distance * 10000);
					if (distance > maximum)
						maximum = (int)distance * 10000;
				}
			}

			for (int j = fingerDirection.size(); j < 5; j++)
			{
				for (int i = 0; i < result.cols; i++)
				{
					r[i][j] = maximum;
				}
			}
			hungarian_t prob2;
			//cout << "Hungarian problem 2" << endl;
			hungarian_init(&prob2, (int*)r, 5, 5, HUNGARIAN_MIN);
			//hungarian_print_rating(&prob2);
			hungarian_solve(&prob2);
			//hungarian_print_assignment(&prob2);
			//cout << "Hungarian solved" << endl;
			energyReverse = 0;
			for (int i = 0; i < 5; i++)
				for (int j = 0; j < fingerDirection.size() && j < 5; j++)
				{
				if (j == prob2.a[i])
				{
					//cout << "finger insert" << i << " row " << j << endl;
					finger_index_flip[j] = i;
					energyReverse += r[i][j];
				}
				}

			hungarian_fini(&prob2);

	/*		if (lastHandOrientation)
				energyReverse += 30.0;
			else
				energy += 30.0;*/
			DBOUT("\n");
			DBOUT(energy);
			DBOUT("\n");
			DBOUT(energyReverse);

			if (energyReverse < energy)
			{
				handReverse = false;
				for (int j = 0; j < fingerDirection.size() && j < 5; j++)
				{
					int i = finger_index_flip[j];
					detected_finger[i] = true;
					//cout << "Finger position " << i << endl;
					Point3d finger_forward = Point3d(result_flip.at<float>(0, i) + finger_scale[i] * fingerDirection[j].x,
						result_flip.at<float>(1, i) + finger_scale[i] * fingerDirection[j].y,
						result_flip.at<float>(2, i) + finger_scale[i] * fingerDirection[j].z);

					//Point3d finger_forward = Point3d(result_flip.at<float>(0, i) ,
					//					result_flip.at<float>(1, i) ,
					//					result.at<float>(2, i));

					assignFingerTip(i, fingerDirection[j], globalRotateReverse, suggestParameter);

					//CameraSpacePoint cntr_cam;
					//DepthSpacePoint cntr_depth;
					//cntr_cam.X = finger_forward.x;
					//cntr_cam.Y = 0.4226*(finger_forward.y - height) - 0.9063*finger_forward.z;
					//cntr_cam.Z = -0.9063*(finger_forward.y - height) - 0.4426*finger_forward.z;
					//m_pCoordinateMapper->MapCameraPointToDepthSpace(cntr_cam, &cntr_depth);
					//circle(color, cv::Point(cntr_depth.X, cntr_depth.Y), 3, cv::Scalar(i*51, 255, 255), -1);
				}

				m[0][0] = globalRotateReverse.at<float>(0, 0);
				m[0][1] = globalRotateReverse.at<float>(0, 1);
				m[0][2] = globalRotateReverse.at<float>(0, 2);

				m[1][0] = globalRotateReverse.at<float>(1, 0);
				m[1][1] = globalRotateReverse.at<float>(1, 1);
				m[1][2] = globalRotateReverse.at<float>(1, 2);

				m[2][0] = globalRotateReverse.at<float>(2, 0);
				m[2][1] = globalRotateReverse.at<float>(2, 1);
				m[2][2] = globalRotateReverse.at<float>(2, 2);
			}
			else
			{
				handReverse = true;
				for (int j = 0; j < fingerDirection.size() && j < 5; j++)
				{
					int i = finger_index[j];
					detected_finger[i] = true;
					//cout << "Finger position " << i << endl;
					Point3d finger_forward = Point3d(result.at<float>(0, i) + finger_scale[i] * fingerDirection[j].x,
						result.at<float>(1, i) + finger_scale[i] * fingerDirection[j].y,
						result.at<float>(2, i) + finger_scale[i] * fingerDirection[j].z);

					/*				Point3d finger_forward = Point3d(result_flip.at<float>(0, i) ,
					result_flip.at<float>(1, i) ,
					result.at<float>(2, i));*/

					assignFingerTip(i, fingerDirection[j], globalRotate, suggestParameter);

					//CameraSpacePoint cntr_cam;
					//DepthSpacePoint cntr_depth;
					//cntr_cam.X = finger_forward.x;
					//cntr_cam.Y = 0.4226*(finger_forward.y - height) - 0.9063*finger_forward.z;
					//cntr_cam.Z = -0.9063*(finger_forward.y - height) - 0.4426*finger_forward.z;

					//m_pCoordinateMapper->MapCameraPointToDepthSpace(cntr_cam, &cntr_depth);
					//circle(color, cv::Point(cntr_depth.X, cntr_depth.Y), 3, cv::Scalar(i * 51, 255, 255), -1);
				}

				m[0][0] = globalRotate.at<float>(0, 0);
				m[0][1] = globalRotate.at<float>(0, 1);
				m[0][2] = globalRotate.at<float>(0, 2);

				m[1][0] = globalRotate.at<float>(1, 0);
				m[1][1] = globalRotate.at<float>(1, 1);
				m[1][2] = globalRotate.at<float>(1, 2);

				m[2][0] = globalRotate.at<float>(2, 0);
				m[2][1] = globalRotate.at<float>(2, 1);
				m[2][2] = globalRotate.at<float>(2, 2);
			}
		
	}

	float factor;
	if (handReverse)
	{
		lastHandOrientation = true;
		factor = 1;
	}
	else
	{
		lastHandOrientation = false;
		factor = 1;
	}

	RtoQ(m, q);
	//cout << factor << endl;

	if (detected_finger[1] == false)
	{
		suggestParameter[7] = PI / 2 * factor;
		suggestParameter[9] = PI / 2 * factor;
		suggestParameter[10] = PI / 2 * factor;
	}
	else if (detected_finger[1] == true)
	{
		suggestParameter[9] = 0;
		suggestParameter[10] = 0;
	}

	if (detected_finger[2] == false)
	{
		suggestParameter[11] = PI / 2 * factor;
		suggestParameter[13] = PI / 2 * factor;
		suggestParameter[14] = PI / 2 * factor;
	}
	else if (detected_finger[2] == true)
	{
		suggestParameter[13] = 0;
		suggestParameter[14] = 0;
	}

	if (detected_finger[3] == false)
	{
		suggestParameter[15] = PI / 2 * factor;
		suggestParameter[17] = PI / 2 * factor;
		suggestParameter[18] = PI / 2 * factor;
	}
	else if (detected_finger[3] == true)
	{
		suggestParameter[17] = 0;
		suggestParameter[18] = 0;
	}

	if (detected_finger[4] == false)
	{
		suggestParameter[19] = PI / 2 * factor;
		suggestParameter[21] = PI / 2 * factor;
		suggestParameter[22] = PI / 2 * factor;
	}
	else if (detected_finger[4] == true)
	{
		suggestParameter[21] = 0;
		suggestParameter[22] = 0;
	}

	if (detected_finger[0] == false)
	{
		suggestParameter[23] = PI / 2 * factor;
		suggestParameter[25] = PI / 2 * factor;
		suggestParameter[26] = PI / 2 * factor;
	}
	else if (detected_finger[0] == true)
	{
		suggestParameter[25] = 0;
		suggestParameter[26] = 0;
	}


	if (detectedPalm)
	{
		suggestParameter[3] = q[0];
		suggestParameter[4] = q[1];
		suggestParameter[5] = q[2];
		suggestParameter[6] = q[3];
	}

}

void Tracker::assignFingerTip(int finger_index, Point3d fingertipDirection, Mat global_rotate, float* suggestParameter)
{
	int rroll = 0;
	int rpitch = 0;
	if (finger_index == 0) rroll = 23, rpitch = 24;
	else if (finger_index == 1) rroll = 7, rpitch = 8;
	else if (finger_index == 2) rroll = 11, rpitch = 12;
	else if (finger_index == 3) rroll = 15, rpitch = 16;
	else if (finger_index == 4) rroll = 19, rpitch = 20;

	Point3d finger_rotate;
	Mat direction(4, 1, CV_32F);
	Mat r(4, 1, CV_32F);

	direction.at<float>(0, 0) = fingertipDirection.x;
	direction.at<float>(1, 0) = fingertipDirection.y;
	direction.at<float>(2, 0) = fingertipDirection.z;
	direction.at<float>(3, 0) = 1;

	r = global_rotate.inv() *direction;

	calculateFingerRotation(Point3d(r.at<float>(0, 0), r.at<float>(1, 0), r.at<float>(2, 0)),
		&suggestParameter[rroll], &suggestParameter[rpitch]);
}

void Tracker::calculateFingerRotation(Point3d finger_direction, float* rpitch, float* rroll)
{
	float norm = sqrt(finger_direction.z*finger_direction.z + finger_direction.y*finger_direction.y);
	Point3d othogonal_no_x = Point3d(0, -finger_direction.z / norm, finger_direction.y / norm);
	if (finger_direction.x == 1 && finger_direction.y == 0 && finger_direction.z == 0)
	{
		othogonal_no_x.y = 0;
		othogonal_no_x.z = 1;
		othogonal_no_x.x = 0;
	}

	if (finger_direction.x == -1 && finger_direction.y == 0 && finger_direction.z == 0)
	{
		othogonal_no_x.y = 0;
		othogonal_no_x.z = 1;
		othogonal_no_x.x = 0;
	}

	Point3d othogonal2 = finger_direction.cross(othogonal_no_x);
	float norm2 = sqrt(othogonal2.y*othogonal2.y + othogonal2.x*othogonal2.x + othogonal2.z*othogonal2.z);
	Point3d othogonal_x = Point3d(othogonal2.x / norm2, othogonal2.y / norm2, othogonal2.z / norm2);

	//cout << othogonal_x.x << " " << finger_direction.x << " " << othogonal_no_x.x << " "  << endl;
	//cout << othogonal_x.y << " " << finger_direction.y << " " << othogonal_no_x.y << " "  << endl;
	//cout << othogonal_x.z << " " << finger_direction.z << " " << othogonal_no_x.z << " " << endl;
	//cout << endl;

	float yaw, roll, pitch;

	pitch = asin(-othogonal_no_x.y);
	float threshold = 0.001; // Hardcoded constant - burn him, he's a witch
	float thes = cos(pitch);
	if (thes > threshold) {
		roll = atan2(othogonal_x.y, finger_direction.y);
		yaw = atan2(othogonal_no_x.x, othogonal_no_x.z);
	}
	else {
		roll = atan2(-finger_direction.x, othogonal_x.x);
		yaw = 0.0;
	}
	//cout << yaw << endl;
	//cout << roll << endl;
	//cout << pitch << endl;

	*rroll = roll;
	*rpitch = pitch;
}

void Tracker::gradient(D3DXVECTOR4* HandSurface, D3DXVECTOR4* HandOut, float* outputParameter)
{

	int parameter = rand() % 20;
	int size;
	int offset;
	if (parameter == 0 || parameter == 1 || parameter == 2)	size = 48, offset = 0;
	else if (parameter == 3 || parameter == 4 || parameter == 5 || parameter == 6) size = 48, offset = 0;
	else if (parameter == 7 || parameter == 8) size = 6, offset = 16;
	else if (parameter == 9 || parameter == 10) size = 4, offset = 18;
	else if (parameter == 11 || parameter == 12) size = 6, offset = 22;
	else if (parameter == 13 || parameter == 14) size = 4, offset = 24;
	else if (parameter == 15 || parameter == 16) size = 6, offset = 28;
	else if (parameter == 17 || parameter == 18) size = 4, offset = 30;
	else if (parameter == 19 || parameter == 20) size = 6, offset = 34;
	else if (parameter == 21 || parameter == 22) size = 4, offset = 36;
	else if (parameter == 23 || parameter == 24) size = 8, offset = 40;
	else if (parameter == 25 || parameter == 26) size = 4, offset = 44;

	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0.0, 1.0);
	bool increasing = true;


	float lastEnergy = 0;
	float v;
	for (int i = 0; i < 256; i++)
	{
		float min_distance = 999999;
		for (int j = offset; j < size + offset; j++)
		{
			float dx = HandOut[j].x - HandSurface[i].x;
			float dy = HandOut[j].y - HandSurface[i].y;
			float dz = HandOut[j].z - HandSurface[i].z;
			float distance = dx*dx + dy*dy + dz*dz;
			if (distance < min_distance)
			{
				min_distance = distance;
			}
		}
		lastEnergy += min_distance;
	}
	v = distribution(generator) * (maxConfig[parameter] - outputParameter[parameter]);
	outputParameter[parameter] += 0.1*v;

	for (int m = 1; m < 10; m++)
	{
		float energy = 0;
		handEncoding(outputParameter, HandOut);
		for (int i = 0; i < 256; i++)
		{
			float min_distance = 999999;
			for (int j = offset; j < size + offset; j++)
			{
				float dx = HandOut[j].x - HandSurface[i].x;
				float dy = HandOut[j].y - HandSurface[i].y;
				float dz = HandOut[j].z - HandSurface[i].z;
				float distance = dx*dx + dy*dy + dz*dz;
				if (distance < min_distance)
				{
					min_distance = distance;
				}
			}
			energy += min_distance;
		}

		if (energy <= lastEnergy)
		{
			if (increasing)
			{
				v = distribution(generator) * (maxConfig[parameter] - outputParameter[parameter]);
				outputParameter[parameter] += 0.3 / m*v;
			}
			else
			{
				v = distribution(generator) * (minConfig[parameter] - outputParameter[parameter]);
				outputParameter[parameter] += 0.3 / m*v;
			}
		}
		else
		{
			if (increasing)
			{
				v = distribution(generator) * (minConfig[parameter] - outputParameter[parameter]);
				outputParameter[parameter] += 0.3 / m*v;
				increasing = false;
			}
			else
			{
				v = distribution(generator) * (maxConfig[parameter] - outputParameter[parameter]);
				outputParameter[parameter] += v*0.3 / m;
				increasing = true;
			}
		}

		lastEnergy = energy;
	}

	//free(outhand);


}

float Tracker::compareHand(float* HandParameter, Mat& depthMap, Point Handcenter, Mat& dist, CameraSpacePoint* HandSurface, D3DXVECTOR4* HandModel, bool optimizing)
{
	DepthSpacePoint* depthPoints = new DepthSpacePoint[48];
	float handCenter_depth = (float)depthMap.at<ushort>(Handcenter.y, Handcenter.x);
	if (handCenter_depth == 0)
	{
		return 999999999;
	}

	float e_d = 0;
	float e_b = 0;
	float e_b_plus = 0;
	float e_b_minus = 0;
	float e_l = 0;
	float result = 0;
	float convert_pixel_to_m = handCenter_depth*tan_deg / 256000.0f;

	CameraSpacePoint* spacePoint = new CameraSpacePoint[48]; // depthSpacePoint
	for (int i = 0; i < 48; i++)
	{
		worldtoCameraSpace(spacePoint[i], HandModel[i].x, HandModel[i].y, HandModel[i].z);
	}
	m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
	for (int i = 0; i < 48; i++)
	{
		if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
		{
			e_b += 200 * convert_pixel_to_m * 200 * convert_pixel_to_m;
		}
		else
		{
			float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
			if (depth != 0)
			{
				depth = depth / 1000.0f; // convert to meter
				float distance = depth - spacePoint[i].Z;
				e_b += max(0.0f, distance)*max(0.0f,distance);
			}
			else
			{
				float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
				e_b += distant * convert_pixel_to_m * distant* convert_pixel_to_m;
			}
		}
	}

	//// + Hand Parameter
	//if (optimizing)
	//{
	//	HandParameter[parameter] += stepSize;
	//	HandEncoding(HandParameter, output);
	//	for (int i = 0; i < 48; i++)
	//	{
	//		spacePoint[i].X = output[i].x;
	//		spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
	//		spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
	//	}
	//	m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
	//	for (int i = 0; i < 48; i++)
	//	{
	//		if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
	//		{
	//			e_b_plus += pow(200 * convert_pixel_to_m, 2.0f);
	//		}
	//		else
	//		{
	//			float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
	//			if (depth != 0)
	//			{
	//				depth = depth / 1000; // convert to meter
	//				e_b_plus += pow(max(0.0f, spacePoint[i].Z - particleRadius - depth), 2.0f);
	//			}
	//			else
	//			{
	//				float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
	//				e_b_plus += pow(distant * convert_pixel_to_m, 2.0f);
	//			}
	//		}
	//	}
	//	// -HandParmeter
	//	HandParameter[parameter] = initialHandParameter - stepSize;
	//	HandEncoding(HandParameter, output);
	//	for (int i = 0; i < 48; i++)
	//	{
	//		spacePoint[i].X = output[i].x;
	//		spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
	//		spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
	//	}
	//	m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
	//	for (int i = 0; i < 48; i++)
	//	{
	//		if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
	//		{
	//			e_b_minus += pow(200 * convert_pixel_to_m, 2.0f);
	//		}
	//		else
	//		{
	//			float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
	//			if (depth != 0)
	//			{
	//				depth = depth / 1000; // convert to meter
	//				e_b_minus += pow(max(0.0f, spacePoint[i].Z - particleRadius - depth), 2.0f);
	//			}
	//			else
	//			{
	//				float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
	//				e_b_minus += pow(distant * convert_pixel_to_m, 2.0f);
	//			}
	//		}
	//	}
	//	if (e_b_plus < e_b_minus && e_b_plus < e_b)
	//	{
	//		HandParameter[parameter] = initialHandParameter + stepSize;
	//		for (int k = 0; k < 10; k++)
	//		{
	//			if (e_b_plus < e_b)
	//			{
	//				e_b = e_b_plus;
	//				HandParameter[parameter] += stepSize;
	//			}
	//			else
	//			{
	//				k = 10;
	//			}
	//			HandEncoding(HandParameter, output);
	//			for (int i = 0; i < 48; i++)
	//			{
	//				spacePoint[i].X = output[i].x;
	//				spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
	//				spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
	//			}
	//			m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
	//			for (int i = 0; i < 48; i++)
	//			{
	//				if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
	//				{
	//					e_b_plus += pow(200 * convert_pixel_to_m, 2.0f);
	//				}
	//				else
	//				{
	//					float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
	//					if (depth != 0)
	//					{
	//						depth = depth / 1000; // convert to meter
	//						e_b_plus += pow(max(0.0f, spacePoint[i].Z - particleRadius - depth), 2.0f);
	//					}
	//					else
	//					{
	//						float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
	//						e_b_plus += pow(distant * convert_pixel_to_m, 2.0f);
	//					}
	//				}
	//			}
	//		}
	//	}
	//	else if (e_b_minus < e_b_plus && e_b_minus < e_b)
	//	{
	//		HandParameter[parameter] = initialHandParameter - stepSize;
	//		for (int k = 0; k < 10; k++)
	//		{
	//			if (e_b_minus < e_b)
	//			{
	//				e_b = e_b_minus;
	//				HandParameter[parameter] -= stepSize;
	//			}
	//			else
	//			{
	//				k = 10;
	//			}
	//			HandEncoding(HandParameter, output);
	//			for (int i = 0; i < 48; i++)
	//			{
	//				spacePoint[i].X = output[i].x;
	//				spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
	//				spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
	//			}
	//			m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
	//			for (int i = 0; i < 48; i++)
	//			{
	//				if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
	//				{
	//					e_b_minus += pow(200 * convert_pixel_to_m, 2.0f);
	//				}
	//				else
	//				{
	//					float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
	//					if (depth != 0)
	//					{
	//						depth = depth / 1000; // convert to meter
	//						e_b_minus += pow(max(0.0f, spacePoint[i].Z - particleRadius - depth), 2.0f);
	//					}
	//					else
	//					{
	//						float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
	//						e_b_minus += pow(distant * convert_pixel_to_m, 2.0f);
	//					}
	//				}
	//			}
	//		}
	//	}
	//}
	//for (int i = 16; i < 40; i++)
	//	for (int j = i + 1; j < 40; j++)
	//	{
	//		float dx = spacePoint[j].X - spacePoint[i].X;
	//		float dy = spacePoint[j].Y - spacePoint[i].Y;
	//		float dz = spacePoint[j].Z - spacePoint[i].Z;
	//		float norm = sqrt(dx*dx + dy*dy + dz*dz);
	//		e_l += pow(max(particleRadius * 3.0f - norm, 0.0f), 2.0f);
	//	}
float angle_finger = -min(HandParameter[16] - HandParameter[20], 0.0f) - min(HandParameter[12] - HandParameter[16], 0.0f) -
min(HandParameter[8] - HandParameter[12], 0.0f);
e_l = angle_finger * angle_finger;

	for (int i = 0; i < 256; i++)
	{
		float min_distance = 999999;
		for (int j = 0; j < 48; j++)
		{
			float dx = spacePoint[j].X - HandSurface[i].X;
			float dy = spacePoint[j].Y - HandSurface[i].Y;
			float dz = spacePoint[j].Z - HandSurface[i].Z;
			float distance = dx*dx + dy*dy + dz*dz;
			if (distance < min_distance)
			{
				min_distance = distance;
			}
		}
		float real_dist = sqrt(min_distance) - particleRadius;
		e_d +=(real_dist*real_dist);
	}
	

	result = (48.0f / 256.0f)*e_d + e_l + e_b;

	free(spacePoint);
	free(depthPoints);
	return result;
}

void Tracker::handEncoding(float* HandParameter, D3DXVECTOR4* output)
{
	//D3DXMATRIX index_offset, middle_offset, ring_offset, pinky_offset, thumb_offset;
	//D3DXMatrixTranslation(&index_offset, -particleRadius*0.25f, particleRadius*1.0f, 0.0f);
	//D3DXMatrixTranslation(&middle_offset, 0.0f, particleRadius*1.2f, 0.0f);
	//D3DXMatrixTranslation(&ring_offset, particleRadius*0.25f, particleRadius*1.0f, 0.0f);
	//D3DXMatrixTranslation(&pinky_offset, particleRadius*0.5f, particleRadius*0.6, 0.0f);
	D3DXMATRIX index_offset, middle_offset, ring_offset, pinky_offset, thumb_offset;
	D3DXMatrixTranslation(&index_offset, 0.0f, particleRadius*1.2f, 0.0f);
	D3DXMatrixTranslation(&middle_offset, 0.0f, particleRadius*1.4f, 0.0f);
	D3DXMatrixTranslation(&ring_offset, 0.0f, particleRadius*1.2f, 0.0f);
	D3DXMatrixTranslation(&pinky_offset, 0.0f, particleRadius, 0.0f);

	D3DXMatrixTranslation(&thumb_offset, 0.0f, particleRadius*1.2, 0.0f);

	//World Coordinate -Palm
	D3DXMATRIX global_translate;
	D3DXMatrixTranslation(&global_translate, HandParameter[0], HandParameter[1], HandParameter[2]);

	D3DXMATRIX global_rotate, global;
	D3DXQUATERNION global_quaternion;
	global_quaternion.x = HandParameter[3];
	global_quaternion.y = HandParameter[4];
	global_quaternion.z = HandParameter[5];
	global_quaternion.w = HandParameter[6];
	D3DXQuaternionNormalize(&global_quaternion, &global_quaternion);
	D3DXMatrixRotationQuaternion(&global_rotate, &global_quaternion);

	global = global_rotate* global_translate;

	// index_finger
	D3DXMATRIX index_rotate[3], index_transform[3], index_final[6], index_position;

	D3DXMatrixRotationYawPitchRoll(&index_rotate[0], 0.0f, HandParameter[7], HandParameter[8]);
	index_transform[0] = index_offset*index_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&index_rotate[1], 0.0f, HandParameter[9], 0.0f);
	index_transform[1] = index_offset * index_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&index_rotate[2], 0.0f, HandParameter[10], 0.0f);
	index_transform[2] = index_offset * index_rotate[2];

	D3DXMatrixTranslation(&index_position, -particleRadius * 2, particleRadius * 4, 0.0f);
	//index_final[0] = index_transform[0] * (index_position* global);
	//index_final[1] = index_offset * index_final[0];
	//index_final[2] = index_offset * index_final[1];
	//index_final[3] = index_transform[1] * index_final[2];
	//index_final[4] = index_offset * index_final[3];
	//index_final[5] = index_transform[2] * index_final[4];

	index_final[0] = index_transform[0] * (index_position* global);
	index_final[1] = index_offset * index_final[0];
	index_final[2] = index_transform[1] * index_final[1];
	index_final[3] = index_offset * index_final[2];
	index_final[4] = index_transform[2] * index_final[3];
	index_final[5] = index_offset  * index_final[4];

	// middle_finger
	D3DXMATRIX middle_rotate[3], middle_transform[3], middle_final[6], middle_position;

	D3DXMatrixRotationYawPitchRoll(&middle_rotate[0], 0.0f, HandParameter[11], HandParameter[12]);
	middle_transform[0] = middle_offset*middle_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&middle_rotate[1], 0.0f, HandParameter[13], 0.0f);
	middle_transform[1] = middle_offset * middle_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&middle_rotate[2], 0.0f, HandParameter[14], 0.0f);
	middle_transform[2] = middle_offset * middle_rotate[2];

	D3DXMatrixTranslation(&middle_position, 0, particleRadius * 4, 0.0f);
	//middle_final[0] = middle_transform[0] * (middle_position* global);
	//middle_final[1] = middle_offset * middle_final[0];
	//middle_final[2] = middle_offset * middle_final[1];
	//middle_final[3] = middle_transform[1] * middle_final[2];
	//middle_final[4] = middle_offset * middle_final[3];
	//middle_final[5] = middle_transform[2] * middle_final[4];

	middle_final[0] = middle_transform[0] * (middle_position* global);
	middle_final[1] = middle_offset * middle_final[0];
	middle_final[2] = middle_transform[1] * middle_final[1];
	middle_final[3] = middle_offset * middle_final[2];
	middle_final[4] = middle_transform[2] * middle_final[3];
	middle_final[5] = middle_offset * middle_final[4];

	// ring_finger
	D3DXMATRIX ring_rotate[3], ring_transform[3], ring_final[6], ring_position;

	D3DXMatrixRotationYawPitchRoll(&ring_rotate[0], 0.0f, HandParameter[15], HandParameter[16]);
	ring_transform[0] = ring_offset*ring_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&ring_rotate[1], 0.0f, HandParameter[17], 0.0f);
	ring_transform[1] = ring_offset * ring_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&ring_rotate[2], 0.0f, HandParameter[18], 0.00);
	ring_transform[2] = ring_offset * ring_rotate[2];

	D3DXMatrixTranslation(&ring_position, particleRadius * 2, particleRadius * 3.5, 0.0f);
	//ring_final[0] = ring_transform[0] * (ring_position* global);
	//ring_final[1] = ring_offset * ring_final[0];
	//ring_final[2] = ring_offset * ring_final[1];
	//ring_final[3] = ring_transform[1] * ring_final[2];
	//ring_final[4] = ring_offset * ring_final[3];
	//ring_final[5] = ring_transform[2] * ring_final[4];

	ring_final[0] = ring_transform[0] * (ring_position* global);
	ring_final[1] = ring_offset * ring_final[0];
	ring_final[2] = ring_transform[1] * ring_final[1];
	ring_final[3] = ring_offset * ring_final[2];
	ring_final[4] = ring_transform[2] * ring_final[3];
	ring_final[5] = ring_offset * ring_final[4];

	//Pinky finger
	D3DXMATRIX pinky_rotate[3], pinky_transform[3], pinky_final[6], pinky_position;

	D3DXMatrixRotationYawPitchRoll(&pinky_rotate[0], 0.0f, HandParameter[19], HandParameter[20]);
	pinky_transform[0] = pinky_offset*pinky_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&pinky_rotate[1], 0.0f, HandParameter[21], 0.0f);
	pinky_transform[1] = pinky_offset * pinky_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&pinky_rotate[2], 0.0f, HandParameter[22], 0.0f);
	pinky_transform[2] = pinky_offset * pinky_rotate[2];

	D3DXMatrixTranslation(&pinky_position, particleRadius * 4, particleRadius * 3, 0.0f);
	pinky_final[0] = pinky_transform[0] * (pinky_position* global);
	pinky_final[1] = pinky_offset * pinky_final[0];
	pinky_final[2] = pinky_transform[1] * pinky_final[1];
	pinky_final[3] = pinky_offset* pinky_final[2];
	pinky_final[4] = pinky_transform[2] * pinky_final[3];
	pinky_final[5] = pinky_offset * pinky_final[4];

	//thumb
	D3DXMATRIX thumb_rotate[3], thumb_transform[3], thumb_final[8], thumb_position;

	D3DXMatrixRotationYawPitchRoll(&thumb_rotate[0], 1.0925, HandParameter[23], HandParameter[24]);
	thumb_transform[0] = thumb_offset*thumb_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&thumb_rotate[1], 0.0f, HandParameter[25], 0.0f);
	thumb_transform[1] = thumb_offset * thumb_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&thumb_rotate[2], 0.0f, HandParameter[26], 0.0f);
	thumb_transform[2] = thumb_offset * thumb_rotate[2];

	D3DXMatrixTranslation(&thumb_position, -particleRadius * 3, -particleRadius * 1.33, -particleRadius);
	thumb_final[0] = thumb_transform[0] * (thumb_position* global);
	thumb_final[1] = thumb_offset * thumb_final[0];
	thumb_final[2] = thumb_offset * thumb_final[1];
	thumb_final[3] = thumb_offset * thumb_final[2];
	thumb_final[4] = thumb_transform[1] * thumb_final[3];
	thumb_final[5] = thumb_offset * thumb_final[4];
	thumb_final[6] = thumb_transform[2] * thumb_final[5];
	thumb_final[7] = thumb_offset * thumb_final[6];

	D3DXVECTOR4 palm[16];

	palm[6] = D3DXVECTOR4(particleRadius*2.0f, -particleRadius * 4, 0.0f, 1.0f);
	palm[7] = D3DXVECTOR4(-particleRadius*2.0f, -particleRadius * 4, 0.0f, 1.0f);
	palm[10] = D3DXVECTOR4(particleRadius*0.67, -particleRadius * 4, 0.0f, 1.0f);
	palm[11] = D3DXVECTOR4(-particleRadius*0.67, -particleRadius * 4, 0.0f, 1.0f);

	palm[14] = D3DXVECTOR4(particleRadius * 3, -particleRadius * 1.33, 0.0f, 1.0f);
	palm[15] = D3DXVECTOR4(-particleRadius * 3, -particleRadius * 1.33, 0.0f, 1.0f);
	palm[2] = D3DXVECTOR4(particleRadius, -particleRadius * 1.33, 0.0f, 1.0f);
	palm[3] = D3DXVECTOR4(-particleRadius, -particleRadius * 1.33, 0.0f, 1.0f);

	palm[12] = D3DXVECTOR4(particleRadius * 3, particleRadius * 1.33, 0.0f, 1.0f);
	palm[13] = D3DXVECTOR4(-particleRadius * 3, particleRadius * 1.33, 0.0f, 1.0f);
	palm[0] = D3DXVECTOR4(particleRadius, particleRadius * 1.33, 0.0f, 1.0f);
	palm[1] = D3DXVECTOR4(-particleRadius, particleRadius * 1.33, 0.0f, 1.0f);

	palm[4] = D3DXVECTOR4(particleRadius * 3, particleRadius * 4, 0.0f, 1.0f);
	palm[5] = D3DXVECTOR4(-particleRadius * 3, particleRadius * 4, 0.0f, 1.0f);
	palm[8] = D3DXVECTOR4(particleRadius, particleRadius * 4, 0.0f, 1.0f);
	palm[9] = D3DXVECTOR4(-particleRadius, particleRadius * 4, 0.0f, 1.0f);

	D3DXVECTOR4 origin = D3DXVECTOR4(0, 0, 0, 1.0);

	for (int i = 0; i < 16; i++)
		D3DXVec4Transform(&output[i], &palm[i], &global);

	for (int i = 0; i < 6; i++)
	{
		D3DXVec4Transform(&output[i + 16], &origin, &index_final[i]);
		D3DXVec4Transform(&output[i + 22], &origin, &middle_final[i]);
		D3DXVec4Transform(&output[i + 28], &origin, &ring_final[i]);
		D3DXVec4Transform(&output[i + 34], &origin, &pinky_final[i]);
	}

	for (int i = 0; i < 8; i++)
		D3DXVec4Transform(&output[i + 40], &origin, &thumb_final[i]);
}

float* Tracker::optimized(float inputConfig[27], float suggestConfig[27], Point HandPosition_depth, Mat depthMap, Mat dist, CameraSpacePoint* HandSurface, bool UseSuggestion)
{
	float* outputConfig = new float[27];
	float x[num_particle_pso][27];
	float xPbest[num_particle_pso][27];
	float pbest[num_particle_pso];
	float labelBest[4] = { 99999 };
	float xLabelBest[4][27];
	float gbest = 999999;
	float v[num_particle_pso][27];
	float averageDistance[num_particle_pso];
	float minDistance;
	int minDistanceIndex;
	int i, j, k, m;

	omp_set_dynamic(0);
	omp_set_num_threads(16);

	//apply to limit search range
	minConfig[0] = inputConfig[0] - 0.1;
	maxConfig[0] = inputConfig[0] + 0.1;
	minConfig[1] = inputConfig[1] - 0.1;
	maxConfig[1] = inputConfig[1] + 0.1;
	minConfig[2] = inputConfig[2] - 0.1;
	maxConfig[2] = inputConfig[2] + 0.1;

	//Generate Particle From the last frame
	std::default_random_engine generator;


	//Set first particle as last frame particle
	if (UseSuggestion)
	{
		for (j = 0; j < 27; j++)
		{
			x[0][j] = inputConfig[j];
			xPbest[0][j] = inputConfig[j];
			outputConfig[j] = inputConfig[j];
			v[0][j] = 0;
			pbest[0] = 9999;

			x[1][j] = suggestConfig[j];
			xPbest[1][j] = suggestConfig[j];
			v[1][j] = 0;
			pbest[1] = 9999;
		}
		// Random position for 3 particle without finger configuration
		for (i = 2; i < 16; i++)
		{
			//x[i][0] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[0];
			//x[i][1] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[1];
			//x[i][2] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[2];
			std::normal_distribution<float> distribution1(suggestConfig[0], 0.05);
			x[i][0] = distribution1(generator);
			std::normal_distribution<float> distribution2(suggestConfig[1], 0.05);
			x[i][1] = distribution2(generator);
			std::normal_distribution<float> distribution3(suggestConfig[2], 0.05);
			x[i][2] = distribution3(generator);

			v[i][0] = 0;
			v[i][1] = 0;
			v[i][2] = 0;

			xPbest[i][0] = x[i][0];
			xPbest[i][1] = x[i][1];
			xPbest[i][2] = x[i][2];

			for (j = 3; j < 7; j++)
			{
				x[i][j] = suggestConfig[j];
				xPbest[i][j] = suggestConfig[j];
				v[i][j] = 0;
			}
			for (j = 7; j < 27; j++)
			{
				x[i][j] = suggestConfig[j];
				xPbest[i][j] = suggestConfig[j];
				v[i][j] = 0;
			}
			pbest[i] = 99999999;
		}
		// Other Particle Random Every thing
		for (i = 16; i < num_particle_pso; i++)
		{
			std::normal_distribution<float> distribution1(inputConfig[0], 0.05);
			x[i][0] = distribution1(generator);
			std::normal_distribution<float> distribution2(inputConfig[1], 0.05);
			x[i][1] = distribution2(generator);
			std::normal_distribution<float> distribution3(inputConfig[2], 0.05);
			x[i][2] = distribution3(generator);
			v[i][0] = 0;
			v[i][1] = 0;
			v[i][2] = 0;
			xPbest[i][0] = x[i][0];
			xPbest[i][1] = x[i][1];
			xPbest[i][2] = x[i][2];
			for (j = 3; j < 7; j++)
			{
				std::normal_distribution<float> distribution4(inputConfig[j], 0.05);
				x[i][j] = distribution4(generator);
				v[i][j] = 0;
				xPbest[i][j] = x[i][j];
			}

			for (j = 7; j < 27; j++)
			{
				std::normal_distribution<float> distribution4(inputConfig[j], 0.2617);

				x[i][j] = distribution4(generator);
				v[i][j] = 0;
				xPbest[i][j] = x[i][j];
			}
			pbest[i] = 999999;
		}
	}
	else
	{
		for (j = 0; j < 27; j++)
		{
			x[0][j] = inputConfig[j];
			xPbest[0][j] = inputConfig[j];
			outputConfig[j] = inputConfig[j];
			v[0][j] = 0;
			pbest[0] = 9999;
		}
		for (i = 1; i < num_particle_pso; i++)
		{
			std::normal_distribution<float> distribution1(inputConfig[0], 0.05);
			x[i][0] = distribution1(generator);
			std::normal_distribution<float> distribution2(inputConfig[1], 0.05);
			x[i][1] = distribution2(generator);
			std::normal_distribution<float> distribution3(inputConfig[2], 0.05);
			x[i][2] = distribution3(generator);
			v[i][0] = 0;
			v[i][1] = 0;
			v[i][2] = 0;
			xPbest[i][0] = x[i][0];
			xPbest[i][1] = x[i][1];
			xPbest[i][2] = x[i][2];
			for (j = 3; j < 7; j++)
			{
				std::normal_distribution<float> distribution4(inputConfig[j], 0.05);
				x[i][j] = distribution4(generator);
				v[i][j] = 0;
				xPbest[i][j] = x[i][j];
			}

			for (j = 7; j < 27; j++)
			{
				std::normal_distribution<float> distribution4(inputConfig[j], 0.2617);

				x[i][j] = distribution4(generator);
				v[i][j] = 0;
				xPbest[i][j] = x[i][j];
			}
			pbest[i] = 999999;
		}
	}
	D3DXVECTOR4* HandWorldSurface = new D3DXVECTOR4[256];
	for (size_t i = 0; i < 256; ++i)
	{
		cameraToWorldSpace(HandSurface[i], &HandWorldSurface[i].x, &HandWorldSurface[i].y, &HandWorldSurface[i].z);
	}
	D3DXVECTOR4**output = new D3DXVECTOR4*[num_particle_pso];
	for (i = 0; i < num_particle_pso; i++)
	{
		output[i] = new D3DXVECTOR4[48];
	}
	//For Each Generation
	for (k = 0; k < num_generation_pso; k++)
	{
#pragma omp parallel for private(j)
		for (i = 0; i < num_particle_pso; i++)
		{
			handEncoding(x[i], output[i]);
			gradient(HandWorldSurface, output[i], x[i]);

			averageDistance[i] = 0.00001;

			for (j = 0; j < 48; j++)
			{
				averageDistance[i] += sqrt(output[i][j].x* output[i][j].x + output[i][j].y* output[i][j].y + output[i][j].z* output[i][j].z);
			}

			averageDistance[i] = averageDistance[i] / 48.0;
		}

		Mat samples(num_particle_pso, 1, CV_32F, &averageDistance);
		//Mat samples(num_particle_pso, 27, CV_32F, &x);
		Mat labels;
		int attempts = 1;
		Mat centers;
		kmeans(samples, 4, labels, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 100, 0.0001), attempts, KMEANS_PP_CENTERS, centers);

		labelBest[0] = 9999999;
		labelBest[1] = 9999999;
		labelBest[2] = 9999999;
		labelBest[3] = 9999999;
		float energy[num_particle_pso];


#pragma omp parallel for private(j)
		for (i = 0; i < num_particle_pso; i++)
		{
			energy[i] = compareHand(x[i], depthMap, HandPosition_depth, dist, HandSurface, output[i], true);
		}

		for (i = 0; i < num_particle_pso; i++)
		{
			if (energy[i] < pbest[i])
			{
				//for (j = 0; j < 27; j++)
				//{
				//	xPbest[i][j] = x[i][j];
				//}
				std::copy(&x[i][0], &x[i][26], xPbest[i]);
				pbest[i] = energy[i];
			}

			int index = labels.at<int>(i, 0);
			if (energy[i] < labelBest[index])
			{
				//for (j = 0; j < 27; j++)
				//{
				//	xLabelBest[index][j] = x[i][j];
				//}
				std::copy(&x[i][0], &x[i][26], xLabelBest[index]);
				labelBest[index] = energy[i];
			}

			if (energy[i] < gbest)
			{
				//for (j = 0; j < 27; j++)
				//{
				//	outputConfig[j] = x[i][j];
				//}
				std::copy(&x[i][0], &x[i][26], outputConfig);
				gbest = energy[i];
			}
		}



		//cout << labels << endl;
		//		labelBest[0] = 9999;
		//		labelBest[1] = 9999;
		//		labelBest[2] = 9999;
		//		labelBest[3] = 9999;
		//
		//
		//#pragma omp parallel for private(j,output)
		//		for (i = 0; i < num_particle_pso; i++)
		//		{
		//			output = new D3DXVECTOR4[48];
		//			HandEncoding(x[i], output);
		//			float energy = compareHand(x[i], depthMap, HandPosition_depth, dist, HandSurface, output, true);
		//			if (energy < pbest[i])
		//			{
		//				for (j = 0; j < 27; j++)
		//				{
		//					xPbest[i][j] = x[i][j];
		//				}
		//				pbest[i] = energy;
		//			}
		//			int index = labels.at<int>(i,0);
		//			if (index != 0 && index != 1 && index != 2 && index != 3)
		//				index = 0;
		//
		//			if (energy < labelBest[index])
		//			{
		//				for (j = 0; j < 27; j++)
		//				{
		//					xLabelBest[index][j] = x[i][j];
		//				}
		//				labelBest[index] = energy;
		//			}
		//
		//			if (energy < gbest)
		//			{
		//				for (j = 0; j < 27; j++)
		//				{
		//					outputConfig[j] = x[i][j];
		//				}
		//				gbest = energy;
		//			}
		//		}
		//
		//
		//		cout << "labels " << endl << labels << endl;
		//Update Function
#pragma omp parallel for private(j)
		for (i = 0; i < num_particle_pso; i++)
		{
			//int index = labels.at<int>(i,0);
			//if (index != 0 && index != 1 && index != 2 && index != 3)
			//	index = 0;
			int index = labels.at<int>(i, 0);

			for (j = 0; j < 27; j++)
			{
				v[i][j] = 0.729843f*(v[i][j] + (float)rand() / float(RAND_MAX)*2.8f *(xPbest[i][j] - x[i][j]) + 1.3f* (float)rand() / (float)RAND_MAX *(xLabelBest[index][j] - x[i][j]));
				x[i][j] = x[i][j] + v[i][j];

				if (x[i][j] > maxConfig[j] && j != 3 && j != 4 && j != 5 && j != 6)
				{
					x[i][j] = maxConfig[j];// -(x[i][j] - maxConfig[j]);
				}
				if (x[i][j] < minConfig[j] && j != 3 && j != 4 && j != 5 && j != 6)
				{
					x[i][j] = minConfig[j];// +(minConfig[j] - x[i][j]);
				}
			}
		}
	}
	return outputConfig;
}

void Tracker::cameraToWorldSpace(CameraSpacePoint camPoint, float* x, float*y, float*z)
{
	// convert from camera space - > world space
	*x = camPoint.Y;
	*y = -cos_deg*camPoint.X - sin_deg*camPoint.Z + height;
	*z = sin_deg*camPoint.X - cos_deg*camPoint.Z;
}

void Tracker::worldtoCameraSpace(CameraSpacePoint &camPoint, float x, float y, float z)
{
	camPoint.X = -cos_deg*(y - height) + sin_deg*z;
	camPoint.Y = x;
	camPoint.Z = -sin_deg*(y - height) - cos_deg*z;
}

int Tracker::returnFrameCount()
{
	return frame;
}

void Tracker::drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale)
{
	double angle;
	double hypotenuse;
	angle = atan2((double)p.y - q.y, (double)p.x - q.x); // angle in radians
	hypotenuse = sqrt((double)(p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
	//    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
	//    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
	// Here we lengthen the arrow by a factor of scale
	q.x = (int)(p.x - scale * hypotenuse * cos(angle));
	q.y = (int)(p.y - scale * hypotenuse * sin(angle));
	line(img, p, q, colour, 1, CV_AA);
	// create the arrow hooks
	p.x = (int)(q.x + 9 * cos(angle + CV_PI / 4));
	p.y = (int)(q.y + 9 * sin(angle + CV_PI / 4));
	line(img, p, q, colour, 1, CV_AA);
	p.x = (int)(q.x + 9 * cos(angle - CV_PI / 4));
	p.y = (int)(q.y + 9 * sin(angle - CV_PI / 4));
	line(img, p, q, colour, 1, CV_AA);
}

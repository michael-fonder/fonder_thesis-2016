/**
 * Author	: Michael Fonder
 * Year		: 2016
 **/

#ifndef UTILS
#define UTILS

#include <opencv2/core/core.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

//TODO : Make verifications (consistency of input arg, eg length vectors)
	
// vec must be of type CV_32FC1

MatExpr crossMat(const Mat &vec);

MatExpr quatLeftComp(const Mat &q);

MatExpr quatRightComp(const Mat &q);

MatExpr quatToProjMat(Mat quat);

MatExpr buildUpdateQuat(const Mat &deltaTheta);

float chiSquared(int d);

struct noiseParameters
{
	cv::Mat IMU_covar;
	cv::Mat Q_IMU;
};

struct Camera
{
	cv::Mat intrisic;//intrinsic parameters
	cv::Mat distCoeffs;
	//deformation matrix
	cv::Mat q_CI; // 4x1 IMU-to-Camera rotation quaternion
	cv::Mat p_CI; // 3x1 Camera position in IMU frame
};

struct ObjectPose
{
	Mat p;
	Mat q;
	Mat v;
	Mat a;
	Mat ang_vel;
};

class Measurement
{
public:
	Mat omega;
	Mat a_I;
	float dT;
	Measurement()
	{
		dT = 0.01; // initialize dT at 100Hz, ie 10 ms
		omega = Mat::zeros(3, 1, CV_32FC1);
		a_I = Mat::zeros(3, 1, CV_32FC1);
	}
};

#endif
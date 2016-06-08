#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "utils.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;
	
// vec must be of type CV_32FC1

/**
* Build a skew matrix from a vector
**/
MatExpr crossMat(const Mat &vec)
{
	Mat vecCross = Mat::zeros(3,3,CV_32FC1);
	vecCross.at<float>(0,1) = -vec.at<float>(2,0);
	vecCross.at<float>(0,2) =  vec.at<float>(1,0);
	vecCross.at<float>(1,0) =  vec.at<float>(2,0);
	vecCross.at<float>(1,2) = -vec.at<float>(0,0);
	vecCross.at<float>(2,0) = -vec.at<float>(1,0);
	vecCross.at<float>(2,1) =  vec.at<float>(0,0);
	MatExpr tmp(vecCross);
	return vecCross+0.0;
}

/**
* Build a matrix from a quaternion which can be used for Hamilton product
**/
MatExpr quatLeftComp(const Mat &q)
{
	Mat vec = q.rowRange(0,3);
	float scalar = q.at<float>(3,0);
	
	Mat qLC = Mat::zeros(4,4, CV_32FC1);
	qLC.rowRange(0,3).colRange(0,3) = Mat::eye(3,3,CV_32FC1)*scalar;
	qLC.rowRange(0,3).colRange(0,3)-= crossMat(vec);
	//qLC.rowRange(0,3).col(3) = vec;
	vec.copyTo(qLC.rowRange(0,3).col(3));
	qLC.row(3).colRange(0,3) = -vec.t();
	qLC.at<float>(3,3) = scalar;
	
	return qLC+0.0;
}

/**
* Build a matrix from a quaternion which can be used for Hamilton product
**/
MatExpr quatRightComp(const Mat &q)
{
	Mat vec = q.rowRange(0,3);
	float scalar = q.at<float>(3,0);
	
	Mat qRC = Mat::zeros(4,4, CV_32FC1);
	qRC.rowRange(0,3).colRange(0,3) = Mat::eye(3,3,CV_32FC1)*scalar;
	qRC.rowRange(0,3).colRange(0,3)+= crossMat(vec);
	//qLC.rowRange(0,3).col(3) = vec;
	vec.copyTo(qRC.rowRange(0,3).col(3));
	qRC.row(3).colRange(0,3) = -vec.t();
	qRC.at<float>(3,3) = scalar;
	
	return qRC+0.0;
}

/**
* Build a projection matrix from a quaternion
**/
MatExpr quatToProjMat(Mat quat)
{
	Mat R;
	
	Eigen::Matrix3f mat3 = Eigen::Quaternionf(quat.at<float>(3,0),quat.at<float>(0,0), quat.at<float>(1,0), quat.at<float>(2,0)).toRotationMatrix();
	eigen2cv(mat3,R);
	
	// NOTE : need the transpose for correct rot mat (error in ref?)
	return R.t();
}

/**
* Build a quaternion from a small rotation
**/
MatExpr buildUpdateQuat(const Mat &deltaTheta)
{
	Mat deltaq = 0.5*deltaTheta;
	Mat updateQuat = Mat::ones(4,1,CV_32FC1);
	
	deltaq.copyTo(updateQuat.rowRange(0,3));
	updateQuat /= float(norm(updateQuat));
	return updateQuat+0.0;
}

/**
* Returns the value for a chi-squared test for a probability of 95% and a degree of freedom d
**/
float chiSquared(int d)
{
	float table[70] = {	3.841,5.991,7.815,9.488,11.070,
						12.592,14.067,15.507,16.919,18.307,
						19.675,21.026,22.362,23.685,24.996,
						26.296,27.587,28.869,30.144,31.410,
						32.671,33.924,35.172,36.415,37.652,
						38.885,40.113,41.337,42.557,43.773,
						44.985,46.194,47.400,48.602,49.802,
						50.998,52.192,53.384,54.572,55.758,
						56.942,58.124,59.304,60.481,61.656,
						62.830,64.001,65.171,66.339,67.505,
						68.669,69.832,70.993,72.153,73.311,
						74.468,75.624,76.778,77.931,79.082,
						80.232,81.381,82.529,83.675,84.821,
						85.965,87.108,88.250,89.391,90.531
						};
	return table[d-1];
}
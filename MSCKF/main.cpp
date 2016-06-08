/**
 * Author	: Michael Fonder
 * Year		: 2016
 **/

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>

#include <unordered_map>
#include <unordered_set>
#include <set>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include "dynamicVectorContainer.hpp"
#include "dynamicVectorReference.hpp"
#include "imuState.hpp"
#include "MSCKF.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;



int main(int argc, char* argv[])
{
	FileStorage fs;
	Mat distCoeffs;
	Mat cameraMatrix;
	
	//******************************************************************************
	
	string filename = "distCoeffs";
	fs.open(filename+".yml", FileStorage::READ);
	fs[filename] >> distCoeffs;
	
	filename = "cameraMatrix";
	fs.open(filename+".yml", FileStorage::READ);
	fs[filename] >> cameraMatrix;
	
	filename = "featurePos";
	fs.open(filename+".yml", FileStorage::READ);
	Mat featurePos;
	fs[filename] >> featurePos;
	cout <<featurePos  << endl;
	
	filename = "data";
	fs.open(filename+".yml", FileStorage::READ);
	Mat data;
	fs[filename] >> data;
	cout << filename+".yml" << endl;
	data = data.t();
	
	Mat ba, bg, IMU_noise, p_CI, q_CI, MSCKF_params, init_uncertainty;
	filename = "parameters";
	fs.open(filename+".yml", FileStorage::READ);
	fs["ba"] >> ba;
	fs["bg"] >> bg;
	fs["IMU_noise"] >> IMU_noise;
	fs["init_uncertainty"] >> init_uncertainty;
	fs["p_CI"] >> p_CI;
	fs["q_CI"] >> q_CI;
	fs["MSCKF_params"] >> MSCKF_params;
	cout << filename+".yml" << endl;
	cout << ba << bg << IMU_noise << p_CI << q_CI << MSCKF_params << endl;
	
	Measurement newMeasurement;
	IMUstate imustate;
	cout << "datatestbegin *********************************" << endl;
	
	VideoCapture capture("flight_cut.avi"); // attempt to open it as a video file or image sequence
//	VideoCapture capture("test_syn.avi"); // attempt to open it as a video file or image sequence
	
	if (!capture.isOpened()){  // returns true if video capturing has been initialized already
		cerr << "Failed to open the video file or image sequence...\n" << endl;
		return 1;
	}
	Mat frame;
	bool finished = false;
	size_t timeIMU=0, timeVideo=0;
	MSCKF filter;
	filter.trueFeaturesPos = featurePos;
	filter.camera.distCoeffs = distCoeffs;
	filter.camera.intrisic = cameraMatrix;
	filter.setCameraParams(p_CI, q_CI);
	filter.setFilterParams(MSCKF_params);
	ba.copyTo(filter.imustate->ba);
	bg.copyTo(filter.imustate->bg);
	filter.imustate->setQ(IMU_noise);
	filter.imustate->resetCovar(init_uncertainty);
	capture >> frame;
	
	Mat log_p = Mat::zeros(1,6,CV_32FC1);
	Mat local = Mat::zeros(1,6,CV_32FC1);
	
	Mat log_q = Mat::zeros(1,7,CV_32FC1);
	Mat localq = Mat::zeros(1,7,CV_32FC1);
	
	while(!finished)
	{
		capture >> frame;
		if (frame.empty())
		{
			finished = true;
			continue;
		}
// 		if(timeVideo > 150)
// 			break;
		
		while(float(timeIMU)*0.01 <= float(timeVideo)/30)
		{
//			data.rowRange(6,9).col(timeIMU).copyTo(filter.truePos); 
//			data.rowRange(9,13).col(timeIMU).copyTo(filter.trueQuat);
			
			data.rowRange(0,3).col(timeIMU).copyTo(newMeasurement.a_I);
			//newMeasurement.a_I.row(0) *= -1;
			data.rowRange(3,6).col(timeIMU).copyTo(newMeasurement.omega);
			//newMeasurement.omega.rowRange(0,2)*=-1.0;
			filter.propagateIMUStateAndCovar(newMeasurement, float(timeIMU)*0.01);
			local.colRange(0,3) = filter.imustate->p_G.t();
			local.at<float>(0,3) = 	filter.imustate->covar.at<float>(12,12);
			local.at<float>(0,4) = 	filter.imustate->covar.at<float>(13,13);
			local.at<float>(0,5) = 	filter.imustate->covar.at<float>(14,14);
			log_p.push_back(local);
			
			localq.colRange(0,4) = filter.imustate->q_IG.t();
			localq.at<float>(0,4) = 	filter.imustate->covar.at<float>(0,0);
			localq.at<float>(0,5) = 	filter.imustate->covar.at<float>(1,1);
			localq.at<float>(0,6) = 	filter.imustate->covar.at<float>(2,2);
			log_q.push_back(localq);
			++timeIMU;
		}
		
		cvtColor(frame, frame, CV_BGR2GRAY);
		if(!(timeVideo%2))
		{
			cout << "augment " << timeVideo << endl;
			filter.augmentState(frame, float(timeVideo)/30);
			cout << "correct " << timeVideo << endl;
			filter.update();
		}
		++timeVideo;
		FileStorage logFilep("log_p.yml", FileStorage::WRITE);
		logFilep << "log_p" << log_p;
		FileStorage logFileq("log_q.yml", FileStorage::WRITE);
		logFileq << "log_q" << log_q;
		FileStorage covar_IMU("covar_IMU.yml", FileStorage::WRITE);
		covar_IMU << "covar_IMU" << filter.imustate->covar;
		
		cout << "Estimated attitude : " << filter.getOrientation().t() << endl;
		cout << "Estimated position : " << filter.getPosition().t() << endl;
		
	}
	
	cout << filter.getPosition() << endl;
	
    	
    return 0;
}
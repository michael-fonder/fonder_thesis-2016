/**
 * Author	: Michael Fonder
 * Year		: 2016
 **/

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <stdint.h>

#include <opencv2/core/core.hpp>

#include "utils.hpp"
#include "imuState.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

IMUstate::IMUstate()
{
	stateLength = 22;
	q_IG = Mat::zeros(4, 1, CV_32FC1);
	q_IG.at<float>(3,0) = 1;
	p_G = Mat::zeros(3, 1, CV_32FC1);
	v_G = Mat::zeros(3, 1, CV_32FC1);
	bg = Mat::zeros(3, 1, CV_32FC1);
	ba = Mat::zeros(3, 1, CV_32FC1);
	C_IG = quatToProjMat(q_IG);
	delay = 0;
	a_G = Mat::zeros(3, 1, CV_32FC1);
	ang_vel = Mat::zeros(3, 1, CV_32FC1);
	prev_a = Mat::zeros(3, 1, CV_32FC1);
	prev_ang_vel = Mat::zeros(3, 1, CV_32FC1);
	Q = Mat::zeros(12,12, CV_32FC1);
	Mat test = Mat::ones(4, 1, CV_32FC1)/3;
	test.at<float>(1,0) = 0;
	test.at<float>(2,0) = 0.5;
	IMU_dT = 0.1;
					
	v = Mat::zeros(3, 1, CV_32FC1);
	
	covar = Mat::zeros(stateLength,stateLength,CV_32FC1);
	float noise = 0.00000001;
	covar.rowRange( 0, stateLength).colRange( 0, stateLength) = Mat::eye(3,3,CV_32FC1)*noise;
	covar.row(15).col(15) = Mat::eye(1,1,CV_32FC1)*0.001;
	covar.rowRange(16,19).colRange(16,19) = Mat::eye(3,3,CV_32FC1)*0.0001;
	covar.rowRange(19,22).colRange(19,22) = Mat::eye(3,3,CV_32FC1)*0.0001;
	noise = 0.000001;
	Q.rowRange(0,3).colRange(0,3) = Mat::eye(3,3,CV_32FC1)*0.00001;
	Q.rowRange(3,6).colRange(3,6) = Mat::eye(3,3,CV_32FC1)*0.00000005;//00001*0.0001;
	Q.rowRange(6,9).colRange(6,9) = Mat::eye(3,3,CV_32FC1)*0.003;
	Q.rowRange(9,12).colRange(9,12) = Mat::eye(3,3,CV_32FC1)*0.0000001;//00001*0.0001;
	
	bg.at<float>(0,0) = 0.03017625;
	bg.at<float>(1,0) = 0.0039264;
	bg.at<float>(2,0) = 0.080586;
	
	ba.at<float>(0,0) = -0.301112;//-0.30913;
	ba.at<float>(1,0) = -0.40392;
	ba.at<float>(2,0) = 0.0021;//0.005176;//0.008526;
	
	quatToProjMat(test);
}

void IMUstate::setQ(const Mat &noise_vec)
{
	Q.rowRange(0,3).colRange(0,3) = Mat::eye(3,3,CV_32FC1)*noise_vec.at<float>(0,0);
	Q.rowRange(3,6).colRange(3,6) = Mat::eye(3,3,CV_32FC1)*noise_vec.at<float>(1,0);
	Q.rowRange(6,9).colRange(6,9) = Mat::eye(3,3,CV_32FC1)*noise_vec.at<float>(2,0);
	Q.rowRange(9,12).colRange(9,12) = Mat::eye(3,3,CV_32FC1)*noise_vec.at<float>(3,0);
}

void IMUstate::resetCovar(const Mat &uncertainty)
{
	covar = Mat::eye(stateLength,stateLength,CV_32FC1);
	covar.rowRange(0,3).colRange(0,3) *= uncertainty.at<float>(0,0);
	covar.rowRange(3,6).colRange(3,6) *= uncertainty.at<float>(3,0);
	covar.rowRange(6,9).colRange(6,9) *= uncertainty.at<float>(2,0);
	covar.rowRange(9,12).colRange(9,12) *= uncertainty.at<float>(4,0);
	covar.rowRange(12,15).colRange(12,15) *= uncertainty.at<float>(1,0);
	
	covar.row(15).col(15) *= uncertainty.at<float>(5,0);
	covar.rowRange(16,19).colRange(16,19) *= uncertainty.at<float>(6,0);
	covar.rowRange(19,22).colRange(19,22) *= uncertainty.at<float>(7,0);
	
}

void IMUstate::propagateState(const Measurement &measurement)
{
	IMU_dT = measurement.dT;
	p_G.copyTo(prev_p_G);
	v_G.copyTo(prev_v_G);		
	
	// Note : biases remain untouched
	
	// Rotation
	Mat omega = (measurement.omega-bg)*measurement.dT;
	ang_vel.copyTo(prev_ang_vel);
	ang_vel = measurement.omega-bg;
	Mat tmp = Mat::zeros(4,4,CV_32FC1);
	tmp.colRange(0,3).rowRange(0,3) = -crossMat(omega);
	tmp.colRange(0,3).row(3) = -omega.t();
	omega.copyTo(tmp.rowRange(0,3).col(3));
	
	q_IG = q_IG + 0.5*(tmp*q_IG);
	q_IG = q_IG/norm(q_IG);
	
	C_IG.copyTo(prev_C_IG);
	C_IG = quatToProjMat(q_IG);
	
	
	// Translation
	a_G.copyTo(prev_a);
	
	Mat a_G_corrected = C_IG.t()*(measurement.a_I-ba);
	a_G_corrected.at<float>(2,0) -= g; // corrections due to earth rotation neglected
	a_G_corrected.copyTo(a_G);
	
	a_G_corrected = 0.5*(a_G+prev_a);
	
	v_G.copyTo(tmp);
	v_G = v_G + a_G_corrected*measurement.dT;
	p_G = p_G + v_G*measurement.dT + 0.5*a_G_corrected*measurement.dT*measurement.dT;
	
}

// Return Phi which is used to propagate MSCKF covar
Mat IMUstate::propagateCovar(const Measurement &measurement)
{
	Mat G = calcG(measurement);
	// Assume IMU state is already up to date
	
	
	Mat Phi = calcPhi(measurement);
	covar = (Phi*covar*Phi.t()) + (Phi*G*Q*G.t()*Phi.t())*measurement.dT*0.5 + G*Q*G.t();
	
	// Enforce PSD of the result. Not mendatory but improves stability
	// Better methods? :
	// https://wis.kuleuven.be/stat/robust/papers/publications-1993/rousseeuwmolenberghs-transformnonpsdcorrelation-co.pdf
	// http://www.mathworks.com/matlabcentral/answers/6057-repair-non-positive-definite-correlation-matrix
	
	size_t size = stateLength;
	for(size_t i=0; i< size; i++)
	{
		for(size_t j=i; j<size; j++)
		{
			if(i==j)
			{
				if(covar.at<float>(i,j)<0)
					covar.at<float>(i,j) = -covar.at<float>(i,j);
			}
			else
			{
				float mean = 0.5*(covar.at<float>(i,j) + covar.at<float>(j,i));
				covar.at<float>(i,j) = mean;
				covar.at<float>(j,i) = mean;
			}
		}
	}
	return Phi;
}

void IMUstate::getDelayedPose(ObjectPose &pose, Mat &Jt, float dT)
{
	float time_diff = dT+delay;
	
	ang_vel.copyTo(pose.ang_vel);
	a_G.copyTo(pose.a);
	
	Mat omega = pose.ang_vel * time_diff;
	Mat tmp = Mat::zeros(4,4,CV_32FC1);
	tmp.colRange(0,3).rowRange(0,3) = -crossMat(omega);
	tmp.colRange(0,3).row(3) = -omega.t();
	omega.copyTo(tmp.rowRange(0,3).col(3));
	
	pose.q = q_IG + 0.5*(tmp*q_IG);
	pose.q = pose.q/norm(pose.q);
	Mat q0 = Mat::zeros(4,1,CV_32FC1);
	q0.at<float>(3,0)=1;
	q0 = q0 + 0.5*(tmp*q0);
	q0 = q0/norm(q0);
	
	pose.a = quatToProjMat(q0)*pose.a;
	pose.v = v_G+pose.a*time_diff;
			
	if(time_diff>0)
		pose.p = p_G + pose.v*time_diff + 0.5*pose.a*time_diff*time_diff;
	else
		pose.p = p_G + pose.v*time_diff - 0.5*pose.a*time_diff*time_diff;
	
	Jt = Mat::zeros(6,1,CV_32FC1);
	Jt.rowRange(0,3) = quatToProjMat(pose.q).t()*pose.ang_vel;
	pose.v.copyTo(Jt.rowRange(3,6));
	
}

MatExpr IMUstate::calcPhi(const Measurement &measurements)
{
	Mat inv_rot = 0.5*(C_IG.t()+prev_C_IG.t());
	Mat a_int = 0.5*(prev_a+a_G);
	float dT = measurements.dT;
	Mat Phi = Mat::eye(stateLength,stateLength,CV_32FC1);
	Phi.rowRange(12,15).colRange(6,9) = Mat::eye(3,3,CV_32FC1)*dT;
	Mat G = Mat::zeros(3,1,CV_32FC1);
	G.at<float>(2,0) = -g;
	
	Mat tmp = dT*(a_int-G);
	Phi.rowRange(6,9).colRange(0,3) = -crossMat(tmp);
	tmp = 0.5*dT*tmp;
	Phi.rowRange(12,15).colRange(0,3) = -crossMat(tmp);
	
	Phi.rowRange(0,3).colRange(3,6) = -dT*inv_rot;
	Phi.rowRange(6,9).colRange(3,6) = 0.5*dT*dT*crossMat(a_G-G)*inv_rot;
	Phi.rowRange(12,15).colRange(3,6) = dT*0.5*Phi.rowRange(6,9).colRange(3,6);
	
	Phi.rowRange(6,9).colRange(9,12) = -dT*inv_rot;
	Phi.rowRange(12,15).colRange(9,12) = 0.5*dT*Phi.rowRange(6,9).colRange(9,12);
	
	return Phi+0.0;
}

MatExpr IMUstate::calcG(const Measurement &measurements)
{
	Mat G = Mat::zeros(stateLength,12,CV_32FC1);
	G.rowRange(0,3).colRange(0,3) = -Mat::eye(3,3,CV_32FC1);
	G.rowRange(3,6).colRange(3,6) = Mat::eye(3,3,CV_32FC1);
	G.rowRange(6,9).colRange(6,9) = -(C_IG.t());
	G.rowRange(9,12).colRange(9,12) = Mat::eye(3,3,CV_32FC1);
	return G+0.0;
}
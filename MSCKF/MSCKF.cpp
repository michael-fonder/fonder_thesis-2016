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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/eigen.hpp>

#include "dynamicVectorContainer.hpp"
#include "dynamicVectorReference.hpp"
#include "utils.hpp"
#include "imuState.hpp"
#include "MSCKF.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;
using namespace Eigen;

#define Matrixfd Matrix<float, Eigen::Dynamic, Eigen::Dynamic>


MSCKF::MSCKF()
{
	imustate = new IMUstate();
	featureId_cnter = 150;
	maxNbrePoses = 24;
	maxNbreFeatures = 50;
	last_IMU_meas_time=0.0;
	
	camera.q_CI = Mat::zeros(4,1,CV_32FC1);
	
	// normal case orientation
	camera.q_CI.row(0) = 0.5;
	camera.q_CI.row(1) = -0.5;
	camera.q_CI.row(2) = 0.5;
	camera.q_CI.row(3) = -0.5;
	
	camProjMat = quatToProjMat(camera.q_CI);
	
	
	camera.p_CI = Mat::zeros(3,1,CV_32FC1);		
	camera.p_CI = camProjMat*camera.p_CI;
	
	im_noise 			= 0.01;
	tracking_quality 	= 0.1;
	tracking_tresh 		= 0.6;
	posEst_tresh 		= 0.0009;
	
}

void MSCKF::setCameraParams(const Mat &p_CI, const Mat &q_CI)
{
	q_CI.copyTo(camera.q_CI);
	camProjMat = quatToProjMat(camera.q_CI);
	camera.p_CI = camProjMat*p_CI;		
}

void MSCKF::setFilterParams(const Mat &params)
{
	im_noise 			= params.at<float>(0,0);
	tracking_quality 	= params.at<float>(1,0);
	tracking_tresh 		= params.at<float>(2,0);
	posEst_tresh 		= params.at<float>(3,0);
}

// Input == last measurement
void MSCKF::propagateIMUStateAndCovar(const Measurement &measurements, float timestamp)
{
	last_IMU_meas_time = timestamp;
	imustate->propagateState(measurements);
	Mat Phi = imustate->propagateCovar(measurements);
	if(cameraPoses.size() > 0)
		imuCamCovar = Phi*imuCamCovar;
}	

/**
* Returns current position estimation
**/
Mat MSCKF::getPosition()
{
	return imustate->p_G;
}

/**
* Returns current attitude estimation
**/
Mat MSCKF::getOrientation()
{
	return imustate->q_IG;
}

/**
* Returns current filter covariance
**/
Mat MSCKF::getCovar()
{
	return imustate->covar;
}

void MSCKF::updateStateAndCovar()
{
	if(features_to_residualize.size() == 0)
		return;
}

// Augment state when new frame available
void MSCKF::augmentState(const Mat &frame, float timestamp)//const Feature &feature)
{
	Mat Jt;
	CameraPose cameraPose;
	imustate->getDelayedPose(cameraPose.pose, Jt, timestamp-last_IMU_meas_time);
	
	
	// store new IMU pose 		
	size_t N = cameraPoses.size();
	cameraPose.pose.p.copyTo(cameraPose.p_CG);
	cameraPose.pose.q.copyTo(cameraPose.q_CG);
	cameraPoses.push_back(cameraPose);
	
	addFrame(frame);
	
	if(points.size()==0) // If no features detected for current frame, remove current camera pose
		posesToPrune.insert(cameraPoses.last());
	
	// Build MSCKF covariance matrix for state augmentation
	size_t L = imustate->stateLength; // just for increased lisibility of the code
	Mat P = Mat::zeros(L+6*N, L+6*N, CV_32FC1);
	(imustate->covar).copyTo(P.rowRange(0,L).colRange(0,L));
	if(N>0)
	{
		camCovar.copyTo(P.rowRange(L, L+6*N).colRange(L,L+6*N));
		transpose(imuCamCovar, P.rowRange(L,L+6*N).colRange(0,L));
		imuCamCovar.copyTo(P.rowRange(0,L).colRange(L,L+6*N));
	}
	
	// Augment state
	Mat J = calcJ(N, Jt, cameraPose, timestamp-last_IMU_meas_time+imustate->delay);
	Mat p_aug = Mat::eye(6+L+6*N,L+6*N, CV_32FC1);
	J.copyTo(p_aug.rowRange(L+6*N,6+L+6*N).colRange(0,L+6*N));
	
	p_aug = p_aug*P*(p_aug.t());
	
	// assign result
	p_aug.rowRange(0,L)			.colRange(0,L)			.copyTo(imustate->covar);
	p_aug.rowRange(L,L+6+6*N)	.colRange(L,L+6+6*N)	.copyTo(camCovar);
	p_aug.rowRange(0,L)			.colRange(L, L+6+6*N)	.copyTo(imuCamCovar);
	
}
	
/**
* Extract and track features for a new incomming frame
* Params:
*	frame	: the new frame
* Returns the number of features tracked
**/
int MSCKF::addFrame(const Mat &frame)
{
	vector<uchar> status;
	vector<float> err;
	vector<float> err1;
	
	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, tracking_quality);
	Size subPixWinSize(10,10), winSize(7,7);
	
	const int MAX_COUNT = 10;
	int discontinued = 0;
	
	if(cameraPoses.size()>1 && points.size()!= 0)
	{
		vector<Point2f> p1(points);
		vector<Point2f> p0r(points);
		
		// doc : http://docs.opencv.org/2.4/modules/video/doc/motion_analysis_and_object_tracking.html#calcopticalflowpyrlk
		calcOpticalFlowPyrLK(previous_frame, frame,points, p1, status, err, Size(39, 39), 4, termcrit, OPTFLOW_USE_INITIAL_FLOW);
		calcOpticalFlowPyrLK(frame, previous_frame,p1, p0r, status, err1, Size(39, 39), 4, termcrit, OPTFLOW_USE_INITIAL_FLOW);
		
		for(size_t i=0; i<points.size(); ++i)
		{
			if(norm(points[i]-p0r[i])>tracking_tresh)
			{
				status[i]= 0;
				discontinued++;
			}
			else
				points[i] = p1[i];
		}
	}
	
	
	int sx = frame.cols, sy= frame.rows;
	
	Mat mask = Mat::zeros(frame.rows, frame.cols,  CV_8UC1)*255;
	rectangle( mask, Point( sx*0.15, sy*0.15 ), Point( sx*0.85, sy*0.85), 255, -1, 8 );
	int size_circle = int(30*float(features.size()-discontinued)/float(maxNbreFeatures));
	for(size_t i=0; i<points.size(); ++i)
		circle(mask, points[i], size_circle, 0, -1);
	
	// Detect new features only if required
	if(features.size()-discontinued < maxNbreFeatures || float(countNonZero(mask))/float(sx*sy)>0.25)// && (cameraPoses.size()-1)%1 == 0)
	{
		
		imshow("mask", mask);
		
		Mat descriptors1;
		vector<KeyPoint> keypoints;
		BRISK extractor(48,3,1.5);
		extractor(frame, mask, keypoints, descriptors1);
		
		for(size_t i=0; i<keypoints.size(); ++i)
		{
			points.push_back(keypoints[i].pt);
			ids.push_back(featureId_cnter);
			status.push_back(1);
			Feature tmp;
			features.insert({featureId_cnter, tmp});
			++featureId_cnter;
		}
	}
	
	// Highlight tracked features in the image
	mask = frame;
	for(size_t i=0; i<points.size(); ++i)
		circle(mask, points[i], 3, 255, -1);
	
	imshow("frame", mask);
	waitKey(1);
		
		
	for(size_t i=0; i<points.size(); i++)
	{
		int x, y;
		x = points[i].x;
		y = points[i].y;
		
		// Discard features too close from the borders of the image
		if(status[i] == 1 && (x<0.15*sx || x>0.85*sx || y<0.15*sy || y>0.85*sy))
		{
			status[i] = 0;
		}
	}
	
	// Add point updates to data structure
	int longest = -1, max_size = 0;
	for(size_t i=0; i<points.size(); )
	{
		if(max_size<features[ids[i]].positions.size())
		{
			longest  = i;
			max_size = features[ids[i]].positions.size();
		}
		
		if(status[i] == 1)
		{
			Mat matPoint(1,1,CV_32FC2);
			float x = 160, y=120;
			matPoint = {points[i].x, points[i].y};
			undistortPoints(matPoint, matPoint,camera.intrisic, camera.distCoeffs);
			int test = ids[i];
			features[ids[i]].positions.push_back(matPoint.reshape(1,2));
			features[ids[i]].cameraPoses.push_back(cameraPoses.last());
			(cameraPoses.last())->featuresId.insert(ids[i]);
			++i;
		}
		else
		{
			features_to_residualize.insert(ids[i]);
			points.erase(points.begin()+i);
			ids.erase(ids.begin()+i);
			status.erase(status.begin()+i);
		}
	}
	
	if(features_to_residualize.size() < 1 && longest >= 0)
	{
		features_to_residualize.insert(ids[longest]);
		points.erase(points.begin()+longest);
		ids.erase(ids.begin()+longest);
		status.erase(status.begin()+longest);
	}
	
	frame.copyTo(previous_frame);
}

/**
* Performs the EKF update step
**/
void MSCKF::update()
{		
	extractVariablesToPrune();
	
	// Exit function if nothing has to be done
	if(features_to_residualize.size() == 0 && posesToPrune.size()==0)
		return;
	
	
	// Assemble covariance matrix
	size_t N = imustate->stateLength + imuCamCovar.cols;
	Mat P = Mat::zeros(N,N,CV_32FC1);
	imustate->covar.copyTo(P.rowRange(0,imustate->stateLength).colRange(0, imustate->stateLength));
	imuCamCovar.copyTo(P.rowRange(0,imustate->stateLength).colRange(imustate->stateLength, N));
	transpose(imuCamCovar, P.rowRange(imustate->stateLength, N).colRange(0, imustate->stateLength));
	camCovar.copyTo(P.rowRange(imustate->stateLength, N).colRange(imustate->stateLength, N));
	
	Mat Ho, ro, Ro;
	
	int accepted_features = 0;
	
	for(std::unordered_set<int32_t>::iterator it = features_to_residualize.begin(); it != features_to_residualize.end(); it++)
	{
		Mat p_f_G, Hoj, Aj;
		int test = *it;
		Feature feature = features.find(*it)->second;
		
		if(feature.cameraPoses.size() < 3)
		{
			continue;
		}
		
		float d = calcGNPosEst(features[*it], p_f_G);
		if(d/float(features[*it].positions.size())>posEst_tresh)
		{
			continue;
		}
		
		calcHoj(p_f_G, feature, Hoj, Aj);
		
		//Compute residuals
		Mat rj(2*feature.positions.size(), 1, CV_32FC1); // NOTE : Previously : Mat rj = Mat::zeros(2*feature.positions.size(), 1, CV_32FC1);
		for(size_t i=0; i<feature.positions.size(); ++i)
		{
			Mat C_CG = camProjMat*quatToProjMat((feature.cameraPoses[i]).q_CG);
			Mat p_f_C = C_CG*(p_f_G-(feature.cameraPoses[i]).p_CG)-camera.p_CI;
			
			rj.at<float>(2*i,  0) = p_f_C.at<float>(0,0)/p_f_C.at<float>(2,0);
			rj.at<float>(2*i+1,0) = p_f_C.at<float>(1,0)/p_f_C.at<float>(2,0);
			rj.rowRange(2*i, 2*i+2) = feature.positions[i] - rj.rowRange(2*i, 2*i+2);
		}
		
		rj = Aj.t()*rj;	
		
		// Mahanalobis gating test
		Mat Rj = Mat::eye(rj.rows, rj.rows, CV_32FC1)*im_noise;
		Mat S = Hoj*P*Hoj.t()+Rj;
		Mat MD;
		Mat tmp3;
		mySolve(S,rj,MD);
		MD = rj.t()*MD;
cout << "MD : " << MD <<chiSquared(rj.rows) << endl;
		if(isnan(abs(MD.at<float>(0,0))) || MD.at<float>(0,0) > chiSquared(rj.rows))
			continue;
		
		// Project matrices on the left nullspace
		Ho.push_back(Hoj);
		
	
		ro.push_back(rj);
		
		// Push Rj on the diag of Ro
		Mat tmp = Mat::zeros(Ro.rows+Rj.rows, Ro.cols+Rj.cols, CV_32FC1);
		Ro.copyTo(tmp.rowRange(0, Ro.rows).colRange(0, Ro.cols));// += Ro;
		Rj.copyTo(tmp.rowRange(Ro.rows, Ro.rows+Rj.rows).colRange(Ro.cols,Ro.cols+Rj.cols));// += Rj;
		tmp.copyTo(Ro);
		
		++accepted_features;
	}
	
	
	if(accepted_features > 0) //Do filter update only if there are some residuals to process
	{
		Mat Q1, Th;
		{	// Compute range space of Ho to speed up following computations
			MatrixXf eigenHo;
			cv2eigen(Ho, eigenHo);
			FullPivHouseholderQR<MatrixXf> qr(eigenHo);
			MatrixXf Q = qr.matrixQ();
			MatrixXf R = qr.matrixQR().triangularView<Eigen::Upper>();
			R = R*qr.colsPermutation().inverse();
			cout << "range space computed"<<endl;
			
			eigen2cv(Q, Q1);
			eigen2cv(R, Th);
		}
		
		// Speed boost by eliminating noise from the residuals
		for(size_t i=0; i<Th.rows; ++i)
		{
			size_t j=0;
			for(; j<Th.cols;++j)
				if(Th.at<float>(i,j)!=0)
					break;
			if(j==Th.cols)
			{
				Th.rowRange(0,i).copyTo(Th);
				Q1.colRange(0,i).copyTo(Q1);
				break;
			}
		}

		Ro = Q1.t()*Ro*Q1;
		Mat rn = Q1.t()*ro;
		rn.copyTo(ro);
		
		
		Mat tmp2 = Th*P*Th.t()+Ro;			
		Mat K;
		{
			MatrixXf A, b;
			cv2eigen(tmp2.t(), A);
			cv2eigen((P*Th.t()).t(), b);
			
			// Choose your own solving method
// 				MatrixXf K2 = A.colPivHouseholderQr().solve(b);
			MatrixXf K2 = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
//				MatrixXf K2 = A.householderQr().solve(b);
			
			eigen2cv(K2, K);
		}
		transpose(K,K);
		
		// State correction term
		Mat deltaX = K*ro;
		cout << "DeltaX : " << endl << deltaX << endl;
		
		
		if(isnan(abs(deltaX.at<float>(0,0))) || isinf(abs(deltaX.at<float>(0,0))))
		{
			cout << "Warning ill conditionned matrices : State correction failed" << endl;
		}
		else
		{
			updateState(deltaX);
			
			// Covariance correction
			Mat tmp = Mat::eye(N,N, CV_32FC1) - K*Th;
			P = tmp*P*tmp.t() + K*Ro*K.t(); // P corrected
			
			P.rowRange(0,imustate->stateLength).colRange(0,imustate->stateLength)  .copyTo(imustate->covar);
			P.rowRange(0, imustate->stateLength).colRange(imustate->stateLength,N) .copyTo(imuCamCovar);
			P.rowRange(imustate->stateLength, N).colRange(imustate->stateLength, N).copyTo(camCovar);
		}
	}
	
	// State pruning
	pruneState();
}

/**
* Computes Jacobian used to augment the state
* Params:
*	N 			: Number of camera poses stored in the state;
*	Jt			: Jacobian of the time delay;
* 	cameraPose 	: camera pose;
* 	timeStep	: timestep between last received IMU measurements and current frame;
* Returns the Jacobian martix
**/
MatExpr MSCKF::calcJ(size_t N, const Mat &Jt, CameraPose &cameraPose, float timeStep)
{
	Mat rotMat = quatToProjMat(cameraPose.pose.q);
	Mat J = Mat::zeros(6, (imustate->stateLength)+6*N, CV_32FC1);
	J.rowRange(0,3).colRange(0,3) = Mat::eye(3,3,CV_32FC1);
	J.rowRange(3,6).colRange(12,15) = Mat::eye(3,3,CV_32FC1);
	J.rowRange(3,6).colRange(6,9) = Mat::eye(3,3,CV_32FC1)*timeStep;
	J.rowRange(0,6).colRange(15,16) = Jt;
	
	return J+0.0;		
}

/**
* Computes Jacobian of a feature track for the state update
* Params:
*	p_f_G 	: feature position in the global frame;
*	feature	: feature track;
* 	Hoj 	: resulting matrix;
* 	Aj		: nullspace of Hfi
**/
void MSCKF::calcHoj(const Mat &p_f_G, const Feature &feature, Mat &Hoj, Mat &Aj) //const &feature?
{
	size_t IMU = imustate->stateLength; // length of IMU state
	size_t N = cameraPoses.size();
	size_t M = feature.positions.size();
	Mat Hfj = Mat::zeros(2*M, 3, CV_32FC1);
	Mat Hxj = Mat::zeros(2*M, IMU+6*N, CV_32FC1);
	
	
	for(size_t i=0; i<feature.positions.size(); i++)
	{
		int camPoseIndex = cameraPoses.getIndex(feature.cameraPoses.at(i));
		Mat p_CG = (feature.cameraPoses[i]).pose.p;
		Mat q_CG = (feature.cameraPoses[i]).pose.q;
		
		// Express feature position in the camera frame
		Mat p_f_C = camProjMat*quatToProjMat((feature.cameraPoses[i]).q_CG)*(p_f_G-(feature.cameraPoses[i]).p_CG)-camera.p_CI;
		Mat C_CG = camProjMat*quatToProjMat(q_CG);
		
		Mat Ji = Mat::zeros(2,3, CV_32FC1);
		Ji.at<float>(0,0) = 1;
		Ji.at<float>(1,1) = 1;
		Ji.at<float>(0,2) = -p_f_C.at<float>(0,0)/p_f_C.at<float>(2,0);
		Ji.at<float>(1,2) = -p_f_C.at<float>(1,0)/p_f_C.at<float>(2,0);
		Ji /= p_f_C.at<float>(2,0);
		
		
		Mat pos_diff = p_f_G-p_CG;
		Mat tmp = Mat::zeros(2,6, CV_32FC1);
		
		// Jacobian of the residual with respect to the feature position
		Hfj.rowRange(2*i, 2*i+2) = Ji*camProjMat*quatToProjMat((feature.cameraPoses[i]).q_CG);
		
		// Jacobian of the residual with respect to the IMU pose
		tmp.colRange(0, 3) = Hfj.rowRange(2*i, 2*i+2)*crossMat(pos_diff);//Ji*C_CG*crossMat(pos_diff);
		tmp.colRange(3, 6) = -Hfj.rowRange(2*i, 2*i+2);//Ji*C_CG;
		tmp.copyTo(Hxj.rowRange(2*i, 2*i+2).colRange(IMU+6*camPoseIndex, IMU+6*camPoseIndex+6));
		
		// Jacobian of the residual with respect to the time delay
		Hxj.rowRange(2*i, 2*i+2).colRange(15, 16) = -Ji*camProjMat*(crossMat((feature.cameraPoses[i]).pose.ang_vel).t())*quatToProjMat((feature.cameraPoses[i]).q_CG)*pos_diff-Ji*C_CG*(feature.cameraPoses[i]).pose.v;
		
		// Jacobian of the residual with respect to the camera pose
		Hxj.rowRange(2*i, 2*i+2).colRange(16, 19) = Ji*camProjMat*crossMat(quatToProjMat(q_CG)*pos_diff);
		Ji.copyTo(Hxj.rowRange(2*i, 2*i+2).colRange(19, 22));
	}
	
	// QR decomposition to have nullspace
	MatrixXf eigenHfj;
	cv2eigen(Hfj, eigenHfj);
	ColPivHouseholderQR<MatrixXf> qr(eigenHfj);
	MatrixXf Q = qr.matrixQ();
	Q = Q.rightCols(eigenHfj.rows()-eigenHfj.cols());
	eigen2cv(Q, Aj);
	
	// projection on the nullspace of the Jacobian of the residual with respect to the feature position
	Hoj = Aj.t()*Hxj;
}

/**
* Builds camera matrix from camera orientation and location
* Params:
*	C 	: rotation matrix;
*	t 	: translation matrix;
* Returns camera matrix
**/	
Mat MSCKF::buildCameraMatrix(const Mat &C, const Mat &t)
{
	Mat matrix = Mat::zeros(3,4, CV_32FC1);
	C.copyTo(matrix.colRange(0,3));
	t.copyTo(matrix.col(3));
	return matrix+0.0;
}


/**
* Solves the AX=b linear system
* Params:
*	A 	: A matrix;
*	b 	: b matrix;
* 	res : matrix to store the result;
**/	
void MSCKF::mySolve(const Mat &A, const Mat &b, Mat &res)
{
	Matrixfd eigA, eigb;
	cv2eigen(A, eigA);
	cv2eigen(b, eigb);
	Matrixfd eigRes = eigA.colPivHouseholderQr().solve(eigb);
	eigen2cv(eigRes, res);
}


/**
* Performs Gauss-Newton regression on a set of points corresponding to a same feature to estimate its 3d location
* Params:
*	feature : the feature whose position needs to be estimated;
*	pos 	: a matrix to store the result;
**/	
float MSCKF::calcGNPosEst(const Feature &feature, Mat &pos)
{
	pos = Mat::zeros(3,1, CV_32FC1);		
	Mat posCop;
	Mat p_IC = camProjMat.t()*camera.p_CI;
	
	{	// get estimate of feature position by triangulation for state initialisation
		Mat point_4d;
		Mat C1 = camProjMat*quatToProjMat(feature.cameraPoses.front().q_CG);
		Mat C2 = camProjMat*quatToProjMat(feature.cameraPoses.back().q_CG);
		Mat t1 = C1*feature.cameraPoses.front().p_CG+camera.p_CI;
		Mat t2 = C2*feature.cameraPoses.back().p_CG+camera.p_CI;
		
		triangulatePoints( buildCameraMatrix(C1, -t1), buildCameraMatrix(C2, -t2),
						feature.positions.front(), feature.positions.back(), point_4d);
		
		for(size_t i=0; i<3; ++i)
			pos.at<float>(i,0) = point_4d.at<float>(i,0)/point_4d.at<float>(3,0);
		
		pos = C1*(pos)-t1;
	}
	
	//Inverse depth parametrisation
	Mat xEst = Mat::ones(3,1, CV_32FC1);
	xEst.at<float>(0,0) = pos.at<float>(0,0)/pos.at<float>(2,0); //alphaBar
	xEst.at<float>(1,0) = pos.at<float>(1,0)/pos.at<float>(2,0); //betaBar
	xEst.at<float>(2,0) /= pos.at<float>(2,0);			 //rhoBar
	
	size_t n = feature.cameraPoses.size();
	int maxIter = 10;
	
	Mat prov;		
	CameraPose cp_1;
	prov = quatLeftComp(camera.q_CI)*feature.cameraPoses.front().q_CG;
	prov = prov/norm(prov);
	prov.copyTo(cp_1.q_CG);
	prov = feature.cameraPoses.front().p_CG + quatToProjMat(cp_1.q_CG).t()*camera.p_CI;
	prov.copyTo(cp_1.p_CG);
	
	
	Mat sqrErr_prev = Mat::zeros(1,1,CV_32FC1);
	Mat sqrErr;
	for(size_t iter=0; iter<maxIter; iter++)
	{
		Mat E = Mat::zeros(2*n, 3, CV_32FC1);
		Mat errorVec = Mat::zeros(2*n,1, CV_32FC1);
		
		for(size_t i=0; i<n; i++)
		{			
			CameraPose cp_i;
			prov = quatLeftComp(camera.q_CI)*feature.cameraPoses[i].q_CG;
			prov = prov/norm(prov);
			prov.copyTo(cp_i.q_CG);
			prov = feature.cameraPoses[i].p_CG + quatToProjMat(cp_i.q_CG).t()*camera.p_CI;
			prov.copyTo(cp_i.p_CG);
			
			// Compute evolution between first and current camera pose	
			Mat C_i1 = quatToProjMat(cp_i.q_CG)*(quatToProjMat(cp_1.q_CG).t());
			Mat t_i1 = quatToProjMat(cp_i.q_CG)*(cp_1.p_CG - cp_i.p_CG);
			
			// Compute residuals
			Mat tmp = Mat::ones(3,1, CV_32FC1);
			xEst.rowRange(0,2).copyTo(tmp.rowRange(0,2));
			Mat h = C_i1*tmp + xEst.at<float>(2,0)*t_i1;
			tmp = Mat::ones(2,1,CV_32FC1);
			tmp.at<float>(0,0) = h.at<float>(0,0)/h.at<float>(2,0);
			tmp.at<float>(1,0) = h.at<float>(1,0)/h.at<float>(2,0);
			
			// Reproject estimated feature location in the 2d plane to compute the residuals
			errorVec.rowRange(2*i,2*i+2) = feature.positions[i]-tmp;
			
			// Form the Jacobian from eq. (39) in TR_MSCKF
			
			//dEdalpha
			E.at<float>(2*i, 0)   = -C_i1.at<float>(0,0)/h.at<float>(2,0) + (h.at<float>(0,0)/(h.at<float>(2,0)*h.at<float>(2,0)))*C_i1.at<float>(2,0);
			E.at<float>(2*i+1, 0) = -C_i1.at<float>(1,0)/h.at<float>(2,0) + (h.at<float>(1,0)/(h.at<float>(2,0)*h.at<float>(2,0)))*C_i1.at<float>(2,0);
			//dEdbeta
			E.at<float>(2*i, 1)   = -C_i1.at<float>(0,1)/h.at<float>(2,0) + (h.at<float>(0,0)/(h.at<float>(2,0)*h.at<float>(2,0)))*C_i1.at<float>(2,1);
			E.at<float>(2*i+1, 1) = -C_i1.at<float>(1,1)/h.at<float>(2,0) + (h.at<float>(1,0)/(h.at<float>(2,0)*h.at<float>(2,0)))*C_i1.at<float>(2,1);
			//dEdrho
			E.at<float>(2*i, 2)   = -t_i1.at<float>(0,0)/h.at<float>(2,0) + (h.at<float>(0,0)/(h.at<float>(2,0)*h.at<float>(2,0)))*t_i1.at<float>(2,0);
			E.at<float>(2*i+1, 2) = -t_i1.at<float>(1,0)/h.at<float>(2,0) + (h.at<float>(1,0)/(h.at<float>(2,0)*h.at<float>(2,0)))*t_i1.at<float>(2,0);
		}
		
		Mat delta;

		mySolve(E.t()*E, E.t()*errorVec, delta);
		xEst -= delta;
		
		sqrErr = 0.5*errorVec.t()*errorVec;
		Mat err = (sqrErr-sqrErr_prev)/n;//sqrErr;
		sqrErr.copyTo(sqrErr_prev);
		
		// Stop if gain in precision is too small
		if(err.at<float>(0,0) < 0.0000001)
			break;			
	}
	
	// Reproject the feature location into the global referential
	Mat tmp = Mat::ones(3,1, CV_32FC1);
	xEst.rowRange(0,2).copyTo(tmp.rowRange(0,2));
	pos = (1/xEst.at<float>(2,0))*(quatToProjMat(cp_1.q_CG).t())*tmp +cp_1.p_CG;
	return sqrErr.at<float>(0,0);
}

/**
* Extracts the data to use for an EKF filter update
**/
void MSCKF::extractVariablesToPrune()
{
	// Prepare features pruning
	for(std::unordered_set<int32_t>::iterator it = features_to_residualize.begin(); it != features_to_residualize.end(); it++)
	{
		Feature feature = features.find(*it)->second;
		for(size_t i=0; i<feature.cameraPoses.size(); ++i)
		{
			feature.cameraPoses[i].featuresId.erase(*it);
			if(feature.cameraPoses[i].featuresId.size() == 0)
				posesToPrune.insert(feature.cameraPoses.at(i));
		}
	}
	
	// Prepare camera states pruning if too much poses in memory
	if(cameraPoses.size()-posesToPrune.size() < maxNbrePoses)
		return;
	
	std::unordered_map<int32_t, int32_t> features_correspondance;
	
	// Going through poses to prune
	for(size_t i=1; i<cameraPoses.size(); i+=3)
	{
		// Going through each feature detected for current pose
		
		for(std::unordered_set<int32_t>::iterator it = cameraPoses[i].featuresId.begin(); it != cameraPoses[i].featuresId.end(); ++it)
		{
			if(features_to_residualize.find(*it) != features_to_residualize.end())
				continue;
			
			if(features_correspondance.find(*it) == features_correspondance.end())
			{
				// Create temporary features to store information of features present in camera poses to prune
				features_correspondance.insert({*it, featureId_cnter});
				int index = features[*it].cameraPoses.getIndex(cameraPoses.at(i));
				Feature feature;
				features.insert({featureId_cnter, feature});
				features[*it].truePos.copyTo(features[featureId_cnter].truePos);
				features_to_residualize.insert(featureId_cnter);
				++featureId_cnter;
				
				features_to_prune.push_back(*it);
			}
			
			// If feature present in previous camstates, transfer information to correct temporary feature
			int id = features_correspondance[*it];
			int index = features[*it].cameraPoses.getIndex(cameraPoses.at(i));
			features[id].cameraPoses.push_back(cameraPoses.at(i));
			features[id].positions.push_back(features[*it].positions[index]);
			features[*it].positionsToPrune.insert(features[*it].positions.at(index));
		}
		posesToPrune.insert(cameraPoses.at(i));
	}
}

/**
* Deletes the data to use for an EKF filter update
**/
void MSCKF::pruneState()
{
	size_t nbre_remaining_states = cameraPoses.size() - posesToPrune.size();
	size_t L = imustate->stateLength;
	size_t N = 6*cameraPoses.size();
	Mat new_imuCamCovar = Mat:: zeros(L,6*nbre_remaining_states, CV_32FC1);
	
	size_t j=0;
	for(size_t i=0; i<cameraPoses.size(); ++i)
	{
		if(posesToPrune.find(cameraPoses.at(i)) != posesToPrune.end())
		{
			N = camCovar.rows-6; //6*(i-j);
			size_t size = camCovar.rows;
			Mat new_camCovar(N, N, CV_32FC1);
			
			if(j!=0)
				camCovar.rowRange(0, 6*j).colRange(0,6*j)			.copyTo(new_camCovar.rowRange(0, 6*j).colRange(0,6*j));
			if(j!=nbre_remaining_states)
			{
				if(j!=0)
				{
				camCovar.rowRange(6*j+6, size).colRange(0,6*j)		.copyTo(new_camCovar.rowRange(6*j, N).colRange(0,6*j));
				camCovar.rowRange(0,6*j).colRange(6*j+6, size)		.copyTo(new_camCovar.rowRange(0,6*j).colRange(6*j, N));
				}
				camCovar.rowRange(6*j+6, size).colRange(6*j+6, size).copyTo(new_camCovar.rowRange(6*j, N).colRange(6*j, N));
			}
			new_camCovar.copyTo(camCovar);
		}
		else
		{
			imuCamCovar.colRange(6*i, 6*i+6).copyTo(new_imuCamCovar.colRange(6*j, 6*j+6));
			++j;
		}
		
		
	}
	new_imuCamCovar.copyTo(imuCamCovar);
	for(std::unordered_set<int32_t>::iterator it = features_to_residualize.begin(); it != features_to_residualize.end(); ++it)
		features.erase(*it);
	for(size_t i=0; i<features_to_prune.size(); ++i)
	{
		int32_t id = features_to_prune[i];
		features[id].positions.remove(features[id].positionsToPrune);
		features[id].cameraPoses.remove(posesToPrune);
		features[id].positionsToPrune.clear();
	}
	
	cameraPoses.remove(posesToPrune);
	posesToPrune.clear();
	features_to_residualize.clear();
	features_to_prune.clear();
}

/**
* Applies the correction step
**/
void MSCKF::updateState(const Mat &deltaX)
{

	// Update IMU state
	imustate->q_IG 	= quatLeftComp( imustate->q_IG) *buildUpdateQuat(deltaX.rowRange(0,3));
	imustate->q_IG 	= imustate->q_IG/norm(imustate->q_IG);
	imustate->bg 	+= deltaX.rowRange(3,6);
	
	imustate->v_G 	+= deltaX.rowRange(6,9);
	imustate->ba 	+= deltaX.rowRange(9,12);
	imustate->p_G 	+= deltaX.rowRange(12,15);
	
	
	imustate->delay += deltaX.at<float>(16,1);
	camera.q_CI 	= quatLeftComp(camera.q_CI) *buildUpdateQuat(deltaX.rowRange(16,19));
	camera.q_CI 	/= float(norm(camera.q_CI));
	camProjMat 		= quatToProjMat(camera.q_CI);
	camera.p_CI 	+= camProjMat*deltaX.rowRange(19,22);
	
	cout << "Estimated time delay : " << imustate->delay << endl;
	
	// Update camera states
	size_t L = imustate->stateLength;
	for(size_t i=0; i<cameraPoses.size(); ++i)
	{
		cameraPoses[i].p_CG += deltaX.rowRange(L+6*i+3, L+6*i+6);
		cameraPoses[i].q_CG = quatLeftComp(cameraPoses[i].q_CG)*buildUpdateQuat(deltaX.rowRange(L+6*i, L+6*i+3));
		cameraPoses[i].q_CG /= float(norm(cameraPoses[i].q_CG));
	}
}
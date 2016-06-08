/**
 * Author	: Michael Fonder
 * Year		: 2016
 **/

#ifndef MSCKF_HPP
#define MSCKF_HPP

#include <unordered_map>
#include <unordered_set>
#include <set>

#include <opencv2/core/core.hpp>

#include "dynamicVectorContainer.hpp"
#include "dynamicVectorReference.hpp"
#include "utils.hpp"
#include "imuState.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;
using namespace Eigen;

	
class MSCKF
{
public:
	
	// Class structure declaration ****************************
	
	struct CameraPose
	{
		Mat p_CG;
		Mat q_CG;
		ObjectPose pose;
		std::unordered_set<int32_t> featuresId;
	};
	
	struct FeaturePosition
	{
		Mat 		position;
		CameraPose 	*cameraPose;
	};
	
	struct Feature
	{
		DynamicVectorContainer<Mat> 		positions;
		DynamicVectorReference<CameraPose> 	cameraPoses;
		std::unordered_set<Mat*> 			positionsToPrune;
		Mat truePos;
	};
	
	// Filter options ******************************************
	
	size_t maxNbrePoses;	// Max number of camera poses in the filter state
	size_t maxNbreFeatures;	// Max number of features for each frame
	
	size_t nbreMaxIter;		// Max number of iterations for the Gauss-Newton regression
	float  precision;		// Precision threshold for stopping Gauss-Newton optimisaion
	
	// Filter state variables **********************************
	
	IMUstate 		*imustate;
	noiseParameters	noiseParams;
	Mat 			camCovar;
	Mat 			imuCamCovar;
	Camera 			camera;
	
	// Internal variables for visual data management *******************
	
	std::unordered_map<int32_t, Feature>	features;
	DynamicVectorContainer<CameraPose>		cameraPoses;	
	std::unordered_set<CameraPose*>			posesToPrune;
	vector<int32_t> 						features_to_prune;
	unordered_set<int32_t> 					features_to_residualize;
	
	// Variables for features tracking ****************************************
	
	vector<Point2f> 	points;
	vector<int32_t> 	ids;
	vector<int32_t> 	features_to_descriptor_matcher;
	int32_t 			featureId_cnter;
	Mat 				last_features_descriptors;
	Mat 				previous_frame;
	
	
	Mat truePos;
	Mat trueQuat;
	Mat trueFeaturesPos;
	
	Mat camProjMat;
	
	// Internal parameters ******************************************************
	float im_noise;
	float tracking_quality;
	float tracking_tresh;
	float posEst_tresh;
	float last_IMU_meas_time;
	
public:
	
	MSCKF();
	void setCameraParams(const Mat &p_CI, const Mat &q_CI);
	void setFilterParams(const Mat &params);
	void propagateIMUStateAndCovar(const Measurement &measurements, float timestamp);
	Mat getPosition();
	Mat getOrientation();
	Mat getCovar();
	void updateStateAndCovar();
	void augmentState(const Mat &frame, float timestamp);//const Feature &feature);
	int addFrame(const Mat &frame);
	void update();

private:

	MatExpr calcJ(size_t N, const Mat &Jt, CameraPose &cameraPose, float timeStep);
	void calcHoj(const Mat &p_f_G, const Feature &feature, Mat &Hoj, Mat &Aj);
	Mat buildCameraMatrix(const Mat &C, const Mat &t);
	void mySolve(const Mat &A, const Mat &b, Mat &res);
	float calcGNPosEst(const Feature &feature, Mat &pos);
	void extractVariablesToPrune();
	void pruneState();
	void updateState(const Mat &deltaX);
};

#endif
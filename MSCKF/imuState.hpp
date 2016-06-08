/**
 * Author	: Michael Fonder
 * Year		: 2016
 **/

#ifndef IMU_STATE
#define IMU_STATE

#include <stdint.h>

#include <opencv2/core/core.hpp>

#include "utils.hpp"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#define g 9.81//9.80665

using namespace cv;
using namespace std;

//TODO : Make verifications (consistency of input arg, eg length vectors)
	
// vec must be of type CV_32FC1

class IMUstate
{
public:
	
	size_t stateLength;
	// Notation _I == IMU referencial, _G == global frame referencial
	Mat q_IG; 	//unit quaternion for rotation from global to IMU referencial
	Mat p_G; 	// IMU position
	Mat v_G; 	// IMU velocity
	Mat bg; 	// bias on gyroscope
	Mat ba; 	// bias on accelerometer
	Mat C_IG; 	// Variable for saving useless operations during state propagations and updates
	float delay;// Time delay between camera and IMU measurements
	
	Mat prev_p_G;
	Mat prev_v_G;
	Mat prev_a;
	Mat a_G;
	Mat ang_vel;
	Mat prev_ang_vel;
	Mat prev_C_IG;
	float IMU_dT;
	Mat covar;
	Mat Q;
	Mat v;
	
	IMUstate();
	void setQ(const Mat &noise_vec);
	void resetCovar(const Mat &uncertainty);
	void propagateState(const Measurement &measurement);
	Mat propagateCovar(const Measurement &measurement);
	void getDelayedPose(ObjectPose &pose, Mat &Jt, float dT);
	
private:
	
	MatExpr calcPhi(const Measurement &measurements);
	MatExpr calcG(const Measurement &measurements);
};

#endif
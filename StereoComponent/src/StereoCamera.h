
#ifndef _CAMERA_CALIBRATION_H_
	#include "CameraCalibration.h"
#endif

#include "InterfaceStereoCamera.hpp"

// use c++11 standard features
#include <functional>
#include <thread>
#include <fstream>

class StereoCamera : public InterfaceStereoCamera {

public:

	// create the interface implementing the virtual constructor
	InterfaceStereoCamera *Create(){ return new StereoCamera(); };
	~StereoCamera();

	void Init();
	int getStereoCameraState();
	void calibrateStereoCamera(string &leftSettingsFile, string &rightSettingsFile);
	void getIntrinsicParameters(cv::OutputArray &IntrinsicParameters);
	void getDistortionParameters(cv::OutputArray &DistortionMatrices);
	void getStereoTransforms(cv::OutputArray &StereoTransforms);
	void getProjectionMatrices(cv::OutputArray &ProjectionMatrices);
	double getVergenceAngle();

	double getFundamentalMatrix(cv::OutputArray &FundamentalMatrix);
	double getEsentialMatrix(cv::OutputArray &EsentialMatrix);

	/// Find a 3D point test method
	void find3DPoint();

private:
	string leftInputSettingsFile;
	string rightInputSettingsFile;
	CameraCalibration leftCamera;
	CameraCalibration rightCamera;
	StereoCamera::StereoHeadState cameraGlobalStatus;


	/// set the camera state
	void setStereoCameraState(int cameraState, int value);

	/// Calibrate the left and right cameras
	void calibrateCameras(string &leftSettingsFile, string &rightSettingsFile);

	/// Read the intrinsic parameter
	void readIntrinsicParameters(cv::OutputArray &IntrinsicParameters);

	/// Find the similar matches between the images of each camera
	void findMatches();

	/// Find the fundamental matrix between the images of the two camera
	void findFundamentalMatrix();

	/// Find the esential matrix between the images of the two cameras
	void findEssentialMatrix();

	/// Find the Rotation and traslation between the two cameras
	void findStereoTransform();

	

};

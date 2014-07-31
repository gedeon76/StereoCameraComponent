
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
	void getCameraUsefulParameters(cameraParameters &cameraUsefulParameters);
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
	int cameraGlobalStatus;


	/// set the camera state
	void setStereoCameraState(int value);

	/// Calibrate the left and right cameras
	void calibrateCameras(string &leftSettingsFile, string &rightSettingsFile);

	/// Read the intrinsic parameters for the left and right cameras
	void readIntrinsicParameters(cv::OutputArray &intrinsicParameters);

	/// Read some useful camera parameters for the left and right cameras
	void readCameraUsefulParameters(cameraParameters &camerausefulParameters);

	/// Read the distortion parameters for the left and right cameras
	void readDistortionParameters(cv::OutputArray &distortionParameters);

	/// Find the similar matches between the images of each camera
	void findMatches();

	/// Find the fundamental matrix between the images of the two camera
	void findFundamentalMatrix();

	/// Find the esential matrix between the images of the two cameras
	void findEssentialMatrix();

	/// Find the Rotation and traslation between the two cameras
	void findStereoTransform();

	

};

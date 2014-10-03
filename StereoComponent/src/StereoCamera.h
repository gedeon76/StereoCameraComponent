


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
	void getIntrinsicParameters(vector<cv::Mat> &IntrinsicParameters);
	void getCameraUsefulParameters(cameraParameters &cameraUsefulParameters);
	void getDistortionParameters(vector<cv::Mat> &DistortionMatrices);
	void getStereoTransforms(vector<cv::Mat> &StereoTransforms);
	void getProjectionMatrices(vector<cv::Mat> &ProjectionMatrices);
	double getVergenceAngle();

	double getFundamentalMatrix(cv::Mat &FundamentalMatrix);
	double getEsentialMatrix(cv::Mat &EsentialMatrix);

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
	void readIntrinsicParameters(vector<cv::Mat> &intrinsicParameters);

	/// Read some useful camera parameters for the left and right cameras
	void readCameraUsefulParameters(cameraParameters &camerausefulParameters);

	/// Read the distortion parameters for the left and right cameras
	void readDistortionParameters(vector<cv::Mat> &distortionParameters);

	/// Find the similar matches between the images of each camera
	void findMatches();

	/// Find the fundamental matrix between the images of the two camera
	void findFundamentalMatrix();

	/// Find the esential matrix between the images of the two cameras
	void findEssentialMatrix();

	/// Find the Rotation and traslation between the two cameras
	void findStereoTransform();

	

};

#include "StereoCamera.h"

//////////////////////////////////////////////////////////////////////////////////////////////
// interface realization(implementation) for the API of this component
//////////////////////////////////////////////////////////////////////////////////////////////


// Initialize method
void StereoCamera::Init()
{
	cameraGlobalStatus = StereoHeadState::STEREO_NOT_CALIBRATED;
}

// get the calibration status
int StereoCamera::getStereoCameraState()
{
	return cameraGlobalStatus;
}

// calibrate the cameras
void StereoCamera::calibrateStereoCamera(string &leftSettingsFile, string &rightSettingsFile) {

	leftInputSettingsFile.assign(leftSettingsFile);
	rightInputSettingsFile.assign(rightSettingsFile);

	// call the calibration
	calibrateCameras(leftInputSettingsFile, rightInputSettingsFile);

}

// get the intrinsic parameters
void StereoCamera::getIntrinsicParameters(cv::OutputArray &IntrinsicParameters)
{
	readIntrinsicParameters(IntrinsicParameters);
}

// get the distortion parameters
void StereoCamera::getDistortionParameters(cv::OutputArray &DistortionMatrices)
{
	readDistortionParameters(DistortionMatrices);
}

// get the transforms between cameras
void StereoCamera::getStereoTransforms(cv::OutputArray &StereoTransforms){}

// get the projection matrix for each camera 
void StereoCamera::getProjectionMatrices(cv::OutputArray &ProjectionMatrices){}

// get the vergence angle
double StereoCamera::getVergenceAngle(){

	double Angle=0;
	return Angle;
}


//get the fundamental matrix relationship
double StereoCamera::getFundamentalMatrix(cv::OutputArray FundamentalMatrix)
{
	double Error = 0;
	return Error;

}

// get the esential matrix relationship
double StereoCamera::getEsentialMatrix(cv::OutputArray EsentialMatrix)
{
	double Error = 0;
	return Error;
}


//////////////////////////////////////////////////////////////////////////////////////////
// internal methods implementation
//////////////////////////////////////////////////////////////////////////////////////////


StereoCamera::~StereoCamera()
{

}

// set the state
void StereoCamera::setStereoCameraState(int cameraState, int value)
{
	cameraState = value;
}

// calibrate the cameras
void StereoCamera::calibrateCameras(string &leftSettingsFile, string &rightSettingsFile)
{
	// start the calibration process
	int value = StereoHeadState::STEREO_NOT_CALIBRATED;	

	// read the settings files for the left and right cameras
	leftCamera.readSettings(leftSettingsFile);
	rightCamera.readSettings(rightSettingsFile);

	// run the calibration process for each camera

	// store a call to the member function used for camera calibration. 
	// This calling will be used as the thread callable function parameter with its arguments
	std::function<void(CameraCalibration,const string&)> threadCalibrateFunction = &CameraCalibration::getImagesAndFindPatterns;

	//create two threads for camera calibration	
	
	const string leftCameraName("leftCamera");
	const string rightCameraName("rightCamera");

	std::thread threadForLeftCalibration(threadCalibrateFunction, leftCamera,leftCameraName);
	std::thread threadForRightCalibration(threadCalibrateFunction, rightCamera, rightCameraName);

	threadForLeftCalibration.join();
	threadForRightCalibration.join();
	
	// set state to calibrated
	value = StereoHeadState::STEREO_CALIBRATED;
	setStereoCameraState(cameraGlobalStatus, value);

}


// read the intrinsic parameters from the calibration results 
void StereoCamera::readIntrinsicParameters(cv::OutputArray &intrinsicParameters)
{
	Mat K_left, K_right;
	vector<Mat> K_matrices;
	
	leftCamera.getIntrinsicMatrix(K_left);
	rightCamera.getIntrinsicMatrix(K_right);

	K_matrices.push_back(K_left);
	K_matrices.push_back(K_right);

	Mat(K_matrices).copyTo(intrinsicParameters);

}

// read the distortion parameters from the calibration results
void StereoCamera::readDistortionParameters(cv::OutputArray &DistortionMatrices)
{
	Mat D_left, D_right;
	vector<Mat> D_matrices;

	leftCamera.getDistortionMatrix(D_left);
	rightCamera.getDistortionMatrix(D_right);

	D_matrices.push_back(D_left);
	D_matrices.push_back(D_right);

	Mat(D_matrices).copyTo(DistortionMatrices);

}


// find matches on the left and right images
void StereoCamera::findMatches() {
	// TODO - implement StereoCamera::findMatches
	throw "Not yet implemented";
}


// find the fundamental matrix
void StereoCamera::findFundamentalMatrix() {
	// TODO - implement StereoCamera::findFundamentalMatrix
	throw "Not yet implemented";
}


// find the esential matrix
void StereoCamera::findEssentialMatrix() {
	// TODO - implement StereoCamera::findEssentialMatrix
	throw "Not yet implemented";
}


// find the transforms between the left and right cameras
void StereoCamera::findStereoTransform() {
	// TODO - implement StereoCamera::findStereoTransform
	throw "Not yet implemented";
}


// find  a 3d point position
void StereoCamera::find3DPoint() {
	// TODO - implement StereoCamera::find3DPoint
	throw "Not yet implemented";
}












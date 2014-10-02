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

	int value = 0;
	leftInputSettingsFile.assign(leftSettingsFile);
	rightInputSettingsFile.assign(rightSettingsFile);

	// call the calibration
	calibrateCameras(leftInputSettingsFile, rightInputSettingsFile);

	// set state to calibrated
	value = StereoHeadState::STEREO_CALIBRATED;
	setStereoCameraState(value);

}

// get the intrinsic parameters
void StereoCamera::getIntrinsicParameters(vector<cv::Mat> &IntrinsicParameters)
{
	readIntrinsicParameters(IntrinsicParameters);
}

// get some useful camera parameters
void StereoCamera::getCameraUsefulParameters(cameraParameters &cameraParameters)
{
	readCameraUsefulParameters(cameraParameters);
}

// get the distortion parameters
void StereoCamera::getDistortionParameters(vector<cv::Mat> &DistortionMatrices)
{
	readDistortionParameters(DistortionMatrices);
}

// get the transforms between cameras
void StereoCamera::getStereoTransforms(vector<cv::Mat> &StereoTransforms){}

// get the projection matrix for each camera 
void StereoCamera::getProjectionMatrices(vector<cv::Mat> &ProjectionMatrices){}

// get the vergence angle
double StereoCamera::getVergenceAngle(){

	double Angle=0;
	return Angle;
}


//get the fundamental matrix relationship
double StereoCamera::getFundamentalMatrix(cv::Mat &FundamentalMatrix)
{
	double Error = 0;
	return Error;

}

// get the esential matrix relationship
double StereoCamera::getEsentialMatrix(cv::Mat &EsentialMatrix)
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
void StereoCamera::setStereoCameraState(int value)
{
	cameraGlobalStatus = value;
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
	std::function<void(CameraCalibration,const string&)>threadCalibrateFunction = &CameraCalibration::getImagesAndFindPatterns;

	//create two threads for camera calibration	
	
	const string leftCameraName("leftCamera");
	const string rightCameraName("rightCamera");

	std::thread threadForLeftCalibration(threadCalibrateFunction, leftCamera, leftCameraName);
	std::thread threadForRightCalibration(threadCalibrateFunction, rightCamera, rightCameraName);

		
	threadForLeftCalibration.join();
	threadForRightCalibration.join();

	// read the results from xml files
	string leftResultsFile("C:/Users/henry/Documents/Visual Studio 2013/Projects/testStereoCameraComponent/testStereoCameraComponent/Calibration_Results_Left_Camera.xml");
	string rightResultsFile("C:/Users/henry/Documents/Visual Studio 2013/Projects/testStereoCameraComponent/testStereoCameraComponent/Calibration_Results_Right_Camera.xml");

	leftCamera.readResults(leftResultsFile);
	rightCamera.readResults(rightResultsFile);

}


// read the intrinsic parameters from the calibration results 
void StereoCamera::readIntrinsicParameters(vector<cv::Mat> &intrinsicParameters)
{
	Mat K_left = Mat::eye(3,3,CV_64F);
	Mat K_right = Mat::eye(3,3,CV_64F);
		
	leftCamera.getIntrinsicMatrix(K_left);
	rightCamera.getIntrinsicMatrix(K_right);

	intrinsicParameters.push_back(K_left);
	intrinsicParameters.push_back(K_right);
	
}

// read some useful parameters from the calibration results
void StereoCamera::readCameraUsefulParameters(cameraParameters &cameraUsefulParameters)
{
	cameraData Data_Left, Data_Right;

	leftCamera.getCameraUsefulParameters(Data_Left);
	rightCamera.getCameraUsefulParameters(Data_Right);

	cameraUsefulParameters.push_back(Data_Left);
	cameraUsefulParameters.push_back(Data_Right);
}

// read the distortion parameters from the calibration results
void StereoCamera::readDistortionParameters(vector<cv::Mat> &DistortionMatrices)
{
	Mat D_left = Mat::zeros(8, 1, CV_64F); 
	Mat D_right = Mat::zeros(8, 1, CV_64F);
	
	leftCamera.getDistortionMatrix(D_left);
	rightCamera.getDistortionMatrix(D_right);

	DistortionMatrices.push_back(D_left);
	DistortionMatrices.push_back(D_right);
	
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












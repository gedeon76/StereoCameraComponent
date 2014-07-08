#include "StereoCamera.h"

//////////////////////////////////////////////////////////////////////////////////////////////
// interface realization(implementation) for the API of this component
//////////////////////////////////////////////////////////////////////////////////////////////

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
void StereoCamera::getIntrinsicParameters(cv::OutputArray IntrinsicParameters)
{
	int i = 0;
}

// get the distortion parameters
void StereoCamera::getDistortionParameters(cv::OutputArray DistortionMatrices){}

// get the transforms between cameras
void StereoCamera::getStereoTransforms(cv::OutputArray StereoTransforms){}

// get the projection matrix for each camera 
void StereoCamera::getProjectionMatrices(cv::OutputArray ProjectionMatrices){}

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


// calibrate the cameras
void StereoCamera::calibrateCameras(string &leftSettingsFile, string &rightSettingsFile)
{
	// start the calibration process
	cameraGlobalStatus = StereoHeadState::STEREO_NOT_CALIBRATED;

	// read the settings files for the left and right cameras
	leftCamera.readSettings(leftSettingsFile);
	rightCamera.readSettings(rightSettingsFile);

	// run the calibration process for each camera
	leftCamera.getImagesAndFindPatterns();
	rightCamera.getImagesAndFindPatterns();

	// set state to calibrated
	cameraGlobalStatus = StereoHeadState::STEREO_CALIBRATED;
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












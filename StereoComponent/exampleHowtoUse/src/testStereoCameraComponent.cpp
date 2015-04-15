
/*
	This project test the StereoCamera Component
	created for the purpose of calibrate a StereoHead
	that uses two logitech c270 webcams

*/
#include "StereoCameraAccess.h"
#include <boost\filesystem.hpp>

using namespace std;
using namespace cv;
using namespace boost::filesystem;

int main(int argc, char ** argv)
{
	// create the component using the factory method pattern
	StereoCameraAccess AccessObject;
	InterfaceStereoCamera *StereoComponent = AccessObject.CreateStereoCamera();

	// Initialize component
	StereoComponent->Init();

	// create a camera state
	InterfaceStereoCamera::StereoHeadState CameraState;
	string leftCameraSettingsFile;
	string rightCameraSettingsFile;

	// define OutputArrays to save the results of the StereoCamera Component
	vector <cv::Mat> IntrinsicParameters;
	vector <cv::Mat> DistortionParameters;
	vector <cv::Mat> StereoTransforms;
	vector <cv::Mat> ProjectionMatrices;
	double scaleFactor = 0;
	double VergenceAngle = 0;
	int CameraCalibrationStatus = 0;
	cameraParameters cameraUsefulParameters;

	double ErrorFundamentalMatrix = 0, ErrorEsentialMatrix = 0;
	cv::Mat FundamentalMatrix;
	cv::Mat EsentialMatrix;
	cv::Mat rotationFactor, traslationFactor;

	// call the methods from the StereoCamera Component

	// check the camera state
	
	CameraCalibrationStatus = StereoComponent->getStereoCameraState();

	if (CameraCalibrationStatus == InterfaceStereoCamera::STEREO_NOT_CALIBRATED )
	{
		// read the .xml configuration files for the cameras
		string myPath;
		string FileName("Left_Setup_c270.xml");

		boost::filesystem::path p{ "/" };
		StereoComponent->getPathForThisFile(FileName, myPath);

		boost::filesystem::path currentPath = boost::filesystem::current_path();
		
		boost::filesystem::directory_iterator it1{ currentPath };
		while (it1 != boost::filesystem::directory_iterator{})
			std::cout << *it1++ << '\n';
		
		boost::filesystem::path pathToSetupFiles = boost::filesystem::path(myPath);
		boost::filesystem::path parentPath= pathToSetupFiles.parent_path();

		leftCameraSettingsFile.assign(parentPath.generic_string() + p.generic_string() + "Left_Setup_c270.xml");
		rightCameraSettingsFile.assign(parentPath.generic_string() + p.generic_string() + "Right_Setup_c270.xml");

		// call the calibration process
		StereoComponent->calibrateStereoCamera(leftCameraSettingsFile,rightCameraSettingsFile);
		
	}

	CameraCalibrationStatus = StereoComponent->getStereoCameraState();


	if (CameraCalibrationStatus == InterfaceStereoCamera::STEREO_CALIBRATED)
	{
		// get the results
		StereoComponent->getCameraUsefulParameters(cameraUsefulParameters);
		StereoComponent->getIntrinsicParameters(IntrinsicParameters);
		StereoComponent->getDistortionParameters(DistortionParameters);
		StereoComponent->getFundamentalMatrix(FundamentalMatrix);
		StereoComponent->getEsentialMatrix(EsentialMatrix);		
		StereoComponent->getStereoTransforms(StereoTransforms);	
		StereoComponent->getProjectionMatrices(ProjectionMatrices);
		
		// perform a tracking test to proof the results
		StereoComponent->getScaleFactor(scaleFactor, rotationFactor, traslationFactor);
		StereoComponent->testCalibrationProcess();
	}
	
	delete StereoComponent;

	return 0;
}
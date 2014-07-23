/*!  \brief

	This class is the access door to the stereo camera interface
	following the factory method pattern design for the API access
	
	the next lines shows an example of how to use this component
	to get access to its functionalities
	
	 \code{.cpp}
	 	 

	//This project test the StereoCamera Component
	//created for the purpose of calibrate a StereoHead
	//that uses two logitech c270 webcams

	#include "StereoCameraAccess.h"

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
		double VergenceAngle = 0;
		int CameraCalibrationStatus = 0;

		double ErrorFundamentalMatrix = 0, ErrorEsentialMatrix = 0;
		cv::Mat FundamentalMatrix;
		cv::Mat EsentialMatrix;

		// call the methods from the StereoCamera Component

		// check the camera state

		CameraCalibrationStatus = StereoComponent->getStereoCameraState();

		if (CameraCalibrationStatus == InterfaceStereoCamera::STEREO_NOT_CALIBRATED)
		{
			// read the .xml configuration files for the cameras
			leftCameraSettingsFile.assign("C:/Code/UMLCode/StereoComponent/src/Left_Setup_c270.xml");
			rightCameraSettingsFile.assign("C:/Code/UMLCode/StereoComponent/src/Right_Setup_c270.xml");

			// call the calibration process
			StereoComponent->calibrateStereoCamera(leftCameraSettingsFile, rightCameraSettingsFile);

		}

		CameraCalibrationStatus = StereoComponent->getStereoCameraState();

		if (CameraCalibrationStatus == InterfaceStereoCamera::STEREO_CALIBRATED)
		{
			// get the results
			StereoComponent->getIntrinsicParameters(IntrinsicParameters);
			StereoComponent->getDistortionParameters(DistortionParameters);
			StereoComponent->getProjectionMatrices(ProjectionMatrices);
			StereoComponent->getStereoTransforms(StereoTransforms);
		}


		delete StereoComponent;

		return 0;
	}

	 \endcode


*/

#include "StereoCamera.h"				// Interface API


class StereoCameraAccess
{
public:
	 /// Create the StereoCamera Component calling this factory method
	 InterfaceStereoCamera *CreateStereoCamera();

};

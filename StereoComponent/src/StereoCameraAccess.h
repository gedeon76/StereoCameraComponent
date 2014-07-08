/*!  \brief

	This class is the access door to the estereo camera interface
	following the factory method pattern design
	
	the next lines shows an example of how to use this component
	to get access to its functionalities
	
	 \code{.cpp}

	 // include header
	 #include "StereoCameraAccess.h"

	 // create the component using the factory method pattern
	 InterfaceStereoCamera  *StereoComponent = StereoCameraAccess::CreateStereoCamera();
	 
	 // create a camera state 
	 StereoComponent::StereoCameraState CameraState;
	 string leftCameraSettingsFile;
	 string rightCameraSettingsFile;

	 // define matrices to save the results of Stereo camera Component
	 cv::OutputArray IntrinsicParameters;
	 cv::OutputArray DistortionParameters;
	 cv::OutputArray StereoTransforms;
	 cv::OutputArray ProjectionMatrices;
	 double VergenceAngle = 0;
	 int CameraCalibrationStatus = 0;
	 
	 double ErrorFundamental=0,ErrorEsential=0;
	 cv::OutputArray FundamentalMatrix;
	 cv::OutputArray EsentialMatrix;



	 // call the component functions to get the different parameters and matrices

	 // call this only one time or when you change the cameras position or rotation
	 CameraCalibrationStatus = StereoComponent->getStereoCameraState();

	 if (CameraCalibrationStatus == CameraState::STEREO_NOT_CALIBRATED){

		// read the .xml configuration files
		leftCameraSettingsFile.assign("Path_to_myLeftCameraSettings.xml");
		rightCameraSettingsFile.assign("Path_to_myRightCameraSettings.xml");

		// call the calibration process
		StereoComponent->calibrateStereoCamera(leftCameraSettingsFile,rightCameraSettingsFile);	
	 }

	 if (CameraCalibrationStatus == CameraState::STEREO_CALIBRATED){
	
		StereoComponent->getIntrinsicParameters(IntrinsicParameters);
		StereoComponent->getDistortionParameters(DistortionParameters);
		StereoComponent->getProjectionMatrices(ProjectionMatrices);
		StereoComponent->getStereoTransforms(StereoTransforms);
		VergenceAngle = StereoComponent->getVergenceAngle();

		ErrorFundamental = StereoComponent->getFundamentalMatrix(FundamentalMatrix);
		ErrorEsential = StereoComponent->getEsentialMatrix(EsentialMatrix);

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

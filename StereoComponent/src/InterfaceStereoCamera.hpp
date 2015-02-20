/* \brief

	This is the Interface for the stereo camera calibration
	this is based on the OpenCV procedure calibration,
	except for the case of the two cameras and some related
	geometric relationships for the two

	This is the Abstract Base Class that defines the interface for
	this component
	
	The API design uses the factory method software pattern
	see : Chapter 3 Patterns of Book API C++ Design by M. Reedy 2011

	some features are:

	1. use of virtual constructors
	2. use a factory method to enable the creation of the API class using a derived class
*/

/*
	PROJECT:	3DPOINTER
	COMPONENT:	STEREOCAMERA
	DATE:		02.06.14
	AUTHOR:		HENRY PORTILLA
	SPECIAL
	THANKS:		GOD

*/

// include opencv dependency
#include <opencv2/core.hpp>
#include "commonStereoComponent.h"
#include "Trackerpoint.h"

class InterfaceStereoCamera {


public:

	enum StereoHeadState
	{
		STEREO_NOT_CALIBRATED = 0,	/**< Indicates that stereo camera has not been calibrated yet */
		STEREO_CALIBRATED = 1000	/**< Indicates that stereo camera has been calibrated */
	};

	/// Not default constructor instead use a virtual constructor
	virtual InterfaceStereoCamera* Create() = 0;
	virtual ~InterfaceStereoCamera(){};

	/// Initializes the component
	virtual void Init() = 0;

	/// get the actual status of the stereo head
	virtual int getStereoCameraState() = 0;

	/// calibrate the stereo head
	/// @param[in]	leftSettingsFile left camera xml configuration file
	/// @param[in]  rightSettingsFile right camera xml configuration file
	virtual void calibrateStereoCamera(string &leftSettingsFile, string &rightSettingsFile) = 0;

	/// get the intrinsic parameters for each camera
	/// @param[in,out] IntrinsicParameters It contains the matrices KLeft and KRight corresponding
	/// to the internal parameters for the left and right cameras
	virtual void getIntrinsicParameters(vector<cv::Mat> &IntrinsicParameters) = 0;

	/// get the distortion parameters for both cameras
	/// @param[in,out] DistortionMatrices It contains the distortion parameters. The OpenCV model uses 
	/// k1,k2,k3 for radial distortion and p1 and p2 for tangencial distortion.
	/// DistortionMatrices contains the parameters for the left and right cameras
	virtual void getDistortionParameters(vector<cv::Mat> &DistortionMatrices) = 0;

	/// get some useful parameters from the intrinsic matrix found after the calibration process
	/// @param[in,out] CameraParameters It contains some useful camera parameters
	/// as image size, sensor information, field of view, aspect ratio, principal point and focal length
	virtual void getCameraUsefulParameters(cameraParameters &CameraUsefulParameters) = 0;

	/// get the Rotation and traslation between the first and second camera
	/// @param[in,out] StereoTransforms It contains the 3x3 rotation matrix  and  3x1 traslation matrix 
	/// transforms between the left and right cameras
	virtual void getStereoTransforms(vector<cv::Mat> &StereoTransforms) = 0;

	/// get the projection matrices for each camera
	/// @param[in,out] ProjectionMatrices It contains the 3x4 projection matrices Pleft and Pright
	/// for each camera
	virtual void getProjectionMatrices(vector<cv::Mat> &ProjectionMatrices) = 0;

	/// get the vergence angle between the left and right cameras
	virtual double getVergenceAngle() = 0;

	/// get the fundamental matrix between the two cameras for the calibration environment
	/// @param[in,out] FundamentalMatrix it contains the 3x3 fundamental Matrix for the current scene
	/// seen by the two cameras
	/// @return the error on the matrix calculation
	virtual double getFundamentalMatrix(cv::Mat &FundamentalMatrix) = 0;

	/// get the esential matrix between the two cameras for the calibration environment
	/// @param[in,out] EsentialMatrix it contains the 3x3 esential matrix for the current scene 
	/// seen by the two cameras
	/// @return the error on the matrix calculation
	virtual double getEsentialMatrix(cv::Mat &EsentialMatrix) = 0;	

	/// get the scale factor needed to know the true values of objects under triangulation
	/// @param[in,out] ScaleFactor it contains the scale factor needed for triangulation calculus
	virtual void getScaleFactor(double &ScaleFactor) = 0;

	/// get the path to a given file
	/// @param[in,out] Filename it contains the name of the file to look for
	/// @param[in,out] pathFound it contains the path where the file was found
	/// @return a flag indicating if the file was found
	virtual bool getPathForThisFile(string &Filename, string &pathFound) = 0;

	/// realize a test to probe the results from this calibration process
	virtual void testCalibrationProcess() = 0;
	
};

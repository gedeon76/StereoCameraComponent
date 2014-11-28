


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
	bool getPathForThisFile(string &Filename, string &pathFound);

	/// Find a 3D point test method
	void find3DPoint();

private:
	boost::filesystem::path pathToSetupFiles, parentPath;
	string leftInputSettingsFile;
	string rightInputSettingsFile;
	CameraCalibration leftCamera;
	CameraCalibration rightCamera;
	cameraData CameraUsefulParametersLeft;
	cameraData CameraUsefulParametersRight;
	vector<cv::Mat> leftCalibrationImageList;
	vector<cv::Mat> rightCalibrationImageList;
	vector<KeyPoint> matchesLeft, matchesRight;
	vector<DMatch> good_matches;
	cv::Mat imageMatches;
	cv::Mat F_Matrix,E_Matrix;
	cv::Mat PLeft, PRight;
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

	/// Get a path for a given file
	bool getFilePath(string &fileName, string &pathFound);

	/// Read the images used for camera calibration
	void getImageUsedFromCalibration(vector<cv::Mat> &leftImageList, vector<cv::Mat> &rightImageList);

	/// Find the similar matches between the images of each camera
	void findMatches();

	/// Find the fundamental matrix between the images of the two camera
	void findFundamentalMatrix(cv::Mat &F_Matrix);

	/// Find the esential matrix between the images of the two cameras
	void findEssentialMatrix(cv::Mat &EsentialMatrix);

	/// Get the Yaw,Pitch,Roll angles from Rotation between the left and right cameras
	void getRotationAnglesFromMatrix(cv::Mat &RotationMatrix, double &Alpha, double &Beta, double &Gamma);

	/// Get the traslation between the left and right cameras
	void getTraslationFromMatrix(cv::Mat &TraslationMatrix, double &X_shift, double &Y_shift, double &Z_shift);

	/// Find the Rotation and traslation between the two cameras
	void findStereoTransform(vector<cv::Mat> &RotationAndTraslation);

	/// Find the projection Matrices from the Essential Matrix
	void findProjectionMatricesFrom_E_Matrix(vector<cv::Mat> &ProjectionMatrices);

	/// Build a proection Matrix
	void build_Projection_Matrix(cv::Mat &P, cv::Mat R, cv::Mat T);

	/// Test if this 3D point is correct, all values must be positive 
	bool test3DPoint(vector<cv::Point3f> pointsToTest);

	/// Normalize points for finding E matrix
	void normalizePoints(cv::Mat K, vector<cv::Point2f> &inputPoints, vector<cv::Point2f> &normalizedPoints);

	/// Print on console the contents of a given Matrix
	void printMatrix(cv::Mat Matrix, string Matrixname);

	
};

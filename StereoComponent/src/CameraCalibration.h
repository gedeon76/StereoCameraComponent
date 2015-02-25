/*! \brief

	This is the camera calibration class
	it is based on the OpenCV 2.4.9 Tutorials
	about camera calibration but written 
	in a class compact form with multithread capacity

*/

#pragma once

// include settings structure class
#include "Settings.h"
#include "Results.h"
#include "commonStereoComponent.h"

// c++11 headers
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

// include boost libraries
#include <boost\filesystem.hpp>

typedef cameraUsefulData cameraData;



class CameraCalibration {



public:
	CameraCalibration();
	//CameraCalibration(const CameraCalibration &camera);	

	~CameraCalibration();

	int readSettings(string &inputSettingsFile);

	// read the results from the calibration process and save it into a FileStorage structure
	int readResults(string &outputResultsFile);

	void getImagesAndFindPatterns(const string &cameraName);

	double computeReprojectionErrors(vector<vector<Point3f>>& objectPoints, 
									vector<vector<Point2f>>& imagePoints, 
									vector<Mat>& rvecs, vector<Mat>& tvecs, 
									Mat& cameraMatrix, Mat& distCoeffs, vector<float>& perViewErrors);

	void calcBoardCornerPositions(Size boardSize, float squareSize, 
									vector<Point3f>& corners, Settings::Pattern patternType);

	bool runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix, 
						Mat& distCoeffs, vector<vector<Point2f>> imagePoints, 
						vector<Mat>& rvecs, vector<Mat>& tvecs, 
						vector<float>& reprojErrs, double& totalAvgErr);

	void saveCameraParams(Settings& s, Size& imageSize, Mat& cameraMatrix, 
						Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs, 
						vector<float>& reprojErrs,
						vector<vector<Point2f>>& imagePoints, double totalAvgErr);

	bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, 
						Mat& distCoeffs, vector<vector<Point2f>> imagePoints);

	// get the intrinsic calibration matrix found for this camera
	void getIntrinsicMatrix(Mat &intrinsicMatrix);

	// get the extrinsic parameters Rx,Ry,Rz, Tx,Ty, Tz for each image
	void getExtrinsicParameters(Mat &extrinsicParameters);

	void getCameraUsefulParameters(cameraData &cameraUsefulParameters);

	// get the distortion parameters found for this camera
	void getDistortionMatrix(Mat &distortionParametersMatrix);

	// get the images used for the calibration process
	void getImagesUsedForCalibration(vector<capturedFrame> &imageList);

	// get the information about circle patterns used to calibration
	// here these points will be used for scale factor estimation
	void getInfoFromCirclePatterns(vector<circlesDataPerImage> &circlesPatternData);

	// get the number of images used for calibration
	int getHowManyImagesWereUsedperCamera();

	// get file path
	bool getPathForThisFile(string &Filename, string &pathFound);

	// get the file path of the calibration results
	void getPathForResults(string &pathToResults);

	// get camera ID
	void getCameraID(int &cameraID);


private:

	Settings s;
	Results calibrationResults;
	string  inputSettingsFile;
	vector<cv::Point2f> circlePoints;
	vector<vector<Point2f> > imagePoints;
	vector<cv::Point3f> circle_Mis_Positions;
	vector<circlePatternInfo> circlePatterns3D_Data;
	vector<circlesDataPerImage> DataFromCirclesPattern;
	Mat patternInformation;
	Mat cameraMatrix, distCoeffs;
	Mat intrinsicK_Matrix, distortionCoefficients;
	Mat extrinsicParametersMatrix;
	Mat savedImage;
	Size imageSize;
	int mode;
	clock_t prevTimestamp;
	const Scalar RED, GREEN;
	const char ESC_KEY = 27;
	cameraData cameraUsefulParameters;	
	int frameCounter;
	int imageCounter;
	boost::filesystem::path currentPath, resultsPath;

	
};

/*! \brief

	This is the camera calibration class
	it is based on the OpenCV 2.4.9 Tutorials
	about camera calibration but written 
	in a class compact form

*/

// include settings structure class
#include "Settings.h"


class CameraCalibration {


public:
	CameraCalibration();

	~CameraCalibration();

	void help();

	int readSettings(string &inputSettingsFile);

	void getImagesAndFindPatterns();

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

private:

	Settings s;
	string  inputSettingsFile;
	vector<vector<Point2f> > imagePoints;
	Mat cameraMatrix, distCoeffs;
	Size imageSize;
	int mode;
	clock_t prevTimestamp;
	const Scalar RED, GREEN;
	const char ESC_KEY = 27;
	
};

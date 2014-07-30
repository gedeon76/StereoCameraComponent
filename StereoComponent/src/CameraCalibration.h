/*! \brief

	This is the camera calibration class
	it is based on the OpenCV 2.4.9 Tutorials
	about camera calibration but written 
	in a class compact form

*/

// include settings structure class
#include "Settings.h"
#include "commonStereoComponent.h"


typedef cameraUsefulData cameraData;

class CameraCalibration {


public:
	CameraCalibration();

	~CameraCalibration();

	int readSettings(string &inputSettingsFile);

	// read the results from the calibration process and save it into a FileStorage structure
	int readResults(string &outputResultsFile) const;

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

	void getCameraUsefulParameters(cameraData &cameraUsefulParameters);

	// get the distortion parameters found for this camera
	void getDistortionMatrix(Mat &distortionParametersMatrix);





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
	cameraData cameraUsefulParameters;		
	
};

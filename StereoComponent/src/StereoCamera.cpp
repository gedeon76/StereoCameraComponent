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
void StereoCamera::getStereoTransforms(vector<cv::Mat> &StereoTransforms){

	findStereoTransform(StereoTransforms);

}

// get the projection matrix for each camera 
void StereoCamera::getProjectionMatrices(vector<cv::Mat> &ProjectionMatrices){



}

// get the vergence angle
double StereoCamera::getVergenceAngle(){

	double Angle=0;
	return Angle;
}


//get the fundamental matrix relationship
double StereoCamera::getFundamentalMatrix(cv::Mat &FundamentalMatrix)
{
	double Error = 0;

	// get from image points from a couple of images from the calibration process
	findFundamentalMatrix(FundamentalMatrix);

	return Error;
}

// get the esential matrix relationship
double StereoCamera::getEsentialMatrix(cv::Mat &EsentialMatrix)
{
	double Error = 0;
	findEssentialMatrix(EsentialMatrix);
	return Error;
}

// get the path to a given file
bool StereoCamera::getPathForThisFile(string &fileName, string &pathFound)
{
	bool found;
	found = getFilePath(fileName, pathFound);
	return found;
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
	string myPath;
	string FileName("Calibration_Results_Left_Camera.xml");	

	boost::filesystem::path p{"/"};
	getPathForThisFile(FileName, myPath);
	pathToSetupFiles = boost::filesystem::path(myPath);
	parentPath = pathToSetupFiles.parent_path();
		
	string leftResultsFile(parentPath.generic_string() + p.generic_string() + "Calibration_Results_Left_Camera.xml");
	string rightResultsFile(parentPath.generic_string() + p.generic_string() + "Calibration_Results_Right_Camera.xml");

	leftCamera.readResults(leftResultsFile);
	rightCamera.readResults(rightResultsFile);

	// save the image used for calibration
	//leftCamera.getImagesUsedForCalibration(leftCalibrationImageList);
	//rightCamera.getImagesUsedForCalibration(rightCalibrationImageList);

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
	leftCamera.getCameraUsefulParameters(CameraUsefulParametersLeft);
	rightCamera.getCameraUsefulParameters(CameraUsefulParametersRight);

	cameraUsefulParameters.push_back(CameraUsefulParametersLeft);
	cameraUsefulParameters.push_back(CameraUsefulParametersRight);
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

// get the path for a given file
bool StereoCamera::getFilePath(string &fileName,string &pathFound)
{
	bool found = false;

	// look for current path
	boost::filesystem::path directory;
	boost::filesystem::path currentPath = boost::filesystem::current_path();

	// get the number of elements of the path
	int pathElementsSize = 0;
	for (boost::filesystem::path::iterator it = currentPath.begin(); it != currentPath.end(); ++it){
		pathElementsSize = pathElementsSize + 1;
	}

	// built the directory for search 2 levels up
	boost::filesystem::path::iterator itToBuildPath = currentPath.begin();
	for (int i = 0; i < (pathElementsSize - 2); i++){
		directory /= *itToBuildPath;
		++itToBuildPath;
	}

	boost::filesystem::path& path = directory;
	const boost::filesystem::path file = fileName;
	const boost::filesystem::recursive_directory_iterator end;
	const boost::filesystem::recursive_directory_iterator dir_iter(directory);

	const auto it = std::find_if(dir_iter,
		end,
		[&file](const boost::filesystem::directory_entry& e)
	{
		return e.path().filename() == file;
	});

	if (it != end){

		path = it->path();
		pathFound = path.generic_string();		// make the path portable
		found = true;
	}
	return found;
}

// read the images used from calibration from the path where it were saved
void StereoCamera::getImageUsedFromCalibration(vector<cv::Mat> &leftImageList, vector<cv::Mat> &rightImageList)
{
	// read the jpg files
	bool foundLeft,foundRight;
	cv::Mat leftCalibrationImage, rightCalibrationImage;
	string myPath;

	for (int k = 1; k <= leftCamera.getHowManyImagesWereUsedperCamera();k++){

		// build the string to be looked for
		string FileNameLeft("Image" + string(std::to_string(k)) + "leftCamera.jpg");
		string FileNameRight("Image" + string(std::to_string(k)) + "rightCamera.jpg");

		// find and read the images
		foundLeft = getFilePath(FileNameLeft, myPath);
		if (foundLeft){
			leftCalibrationImage = cv::imread(myPath, IMREAD_COLOR);
			leftCalibrationImageList.push_back(leftCalibrationImage);
		}

		foundRight = getFilePath(FileNameRight, myPath);
		if (foundRight){
			rightCalibrationImage = cv::imread(myPath, IMREAD_COLOR);
			rightCalibrationImageList.push_back(rightCalibrationImage);
		}
	}	
	

}

// find matches on the left and right images
void StereoCamera::findMatches() {
	
	// get the images	
	Mat imageLeft, imageRight;

	getImageUsedFromCalibration(leftCalibrationImageList, rightCalibrationImageList);

	imageLeft = leftCalibrationImageList.front();
	imageRight = rightCalibrationImageList.front();

	// here we are going to use the A-KAZE detector and descriptor
	// described in the article
	// Fast explicit diffussion for accelerated features in nonlinear scale spaces
	// BMVC.2013  Pablo Alcantarilla et al

	vector<KeyPoint> keyPointsLeft, KeyPointsRigth;
	Mat descriptorsLeft, descriptorsRigth;
	
	AKAZE akaze;

	akaze(imageLeft, noArray(), keyPointsLeft, descriptorsLeft);
	akaze(imageRight, noArray(), KeyPointsRigth, descriptorsRigth);

	// matcher
	BFMatcher matcherBruteForce(NORM_HAMMING);
	vector<vector<DMatch> > matches;
	matcherBruteForce.knnMatch(descriptorsLeft, descriptorsRigth, matches, 2);

	// find correct matches
	for (size_t i = 0; i < matches.size(); i++)
	{
		DMatch currentMatch = matches[i][0];
		float distance1 = matches[i][0].distance;
		float distance2 = matches[i][1].distance;

		if (distance1 < match_ratio*distance2)
		{
			matchesLeft.push_back(keyPointsLeft[currentMatch.queryIdx]);
			matchesRight.push_back(KeyPointsRigth[currentMatch.trainIdx]);
			good_matches.push_back(currentMatch);
		}
	}

	// draw the results
	imageMatches;
	drawMatches(imageLeft, keyPointsLeft, imageRight, KeyPointsRigth, good_matches, imageMatches, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imshow("Matching witk AKAZE Features", imageMatches);
	imwrite("AKAZEmatches.jpg", imageMatches);	
}


// find the fundamental matrix
void StereoCamera::findFundamentalMatrix(cv::Mat &F_MatrixExtern) {
	
	// find matches between an image pair
	vector<cv::Point2f> leftPoints, rightPoints;
	findMatches();

	// select the RANSAC method to calculate the matrix
	KeyPoint::convert(matchesLeft, leftPoints);
	KeyPoint::convert(matchesRight, rightPoints);
	F_Matrix = findFundamentalMat(leftPoints,rightPoints,FM_RANSAC,3,0.99);
	F_Matrix.copyTo(F_MatrixExtern);
}


// find the esential matrix
void StereoCamera::findEssentialMatrix(cv::Mat &EsentialMatrix) {
	
	cv::Mat tmpResult;
	cv::Mat K_left = Mat::eye(3, 3, CV_64F);
	cv::Mat K_right = Mat::eye(3, 3, CV_64F);

	leftCamera.getIntrinsicMatrix(K_left);
	rightCamera.getIntrinsicMatrix(K_right);

	gemm(K_right,F_Matrix,1.0,noArray(),0.0,tmpResult,GEMM_1_T);
	gemm(tmpResult,K_left,1.0,noArray(),0.0,E_Matrix,GEMM_1_T);
	E_Matrix.copyTo(EsentialMatrix);

}


// get the rotation angles from a rotation matrix
void StereoCamera::getRotationAnglesFromMatrix(cv::Mat &rotationMatrix,double &Alpha,double &Beta,double &Gamma){

	// the method used here is from the notes from the web of Steve LaValle curse on Planning Algorithms
	double R11, R21, R31, R32, R33;

	R11 = rotationMatrix.at<double>(0,0);
	R21 = rotationMatrix.at<double>(1,0);
	R31 = rotationMatrix.at<double>(2,0);
	R32 = rotationMatrix.at<double>(2,1);
	R33 = rotationMatrix.at<double>(2,2);

	Alpha = std::atan2(R21, R11);
	Beta = std::atan2(-R31,hypot(R32,R33));
	Gamma = std::atan2(R32,R33);
}

// find the transforms between the left and right cameras
void StereoCamera::findStereoTransform(vector<cv::Mat> &RotationAndTraslation) {

	
	cv::Mat Rotation, traslation;
	cv::Mat RotationVector;
	vector<cv::Point2f> leftPoints, rightPoints;

	// get an average value for the focal lengths and principal points
	double focalLength = (CameraUsefulParametersLeft.focalLength + CameraUsefulParametersRight.focalLength) / 2;
	cv::Point2d principalPoint((CameraUsefulParametersLeft.principalPointX + CameraUsefulParametersRight.principalPointX)/2,
								(CameraUsefulParametersLeft.principalPointY + CameraUsefulParametersRight.principalPointY)/2);

	// perform a cheirality check to know the positive depth from points
	// some explanation about the esential matrix relationships can be found on the Zisermman book section 9.6
	// there is a section about the possibles poses from a given Esential Matrix
	// So it seems that openCV 3.0 has support to probe the correct pose option
	KeyPoint::convert(matchesLeft, leftPoints);
	KeyPoint::convert(matchesRight, rightPoints);
	recoverPose(E_Matrix, leftPoints, rightPoints, Rotation, traslation, focalLength, principalPoint);

	RotationAndTraslation.push_back(Rotation);
	RotationAndTraslation.push_back(traslation);

	// get axis of rotation
	double Alpha, Beta, Gamma;
	getRotationAnglesFromMatrix(Rotation, Alpha, Beta, Gamma);

}


// find  a 3d point position
void StereoCamera::find3DPoint() {
	// TODO - implement StereoCamera::find3DPoint
	throw "Not yet implemented";
}












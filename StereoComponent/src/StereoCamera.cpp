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

	findProjectionMatricesFrom_E_Matrix(ProjectionMatrices);

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

// perform a test for the results from the calibration
void StereoCamera::testCalibrationProcess(){

	trackTestPointer();
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
	int levelUp = 2;
	boost::filesystem::path::iterator itToBuildPath = currentPath.begin();
	for (int i = 0; i < (pathElementsSize - levelUp); i++){
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
	bool fileFound;
	string pathToFile;
	Mat imageLeft, imageRight;
	
	leftCalibrationImageList.clear();
	rightCalibrationImageList.clear();
	getImageUsedFromCalibration(leftCalibrationImageList, rightCalibrationImageList);

	// choose the middle image pair to find the matches
	int choosedOne = std::abs(std::round((leftCalibrationImageList.size() + rightCalibrationImageList.size()) / 4));

	imageLeft = leftCalibrationImageList.at(choosedOne);
	imageRight = rightCalibrationImageList.at(choosedOne);

	// here we are going to use the A-KAZE detector and descriptor
	// described in the article
	// Fast explicit diffussion for accelerated features in nonlinear scale spaces
	// BMVC.2013  Pablo Alcantarilla et al

	vector<KeyPoint> keyPointsLeft, KeyPointsRigth;
	Mat descriptorsLeft, descriptorsRigth;
	
	cv::Ptr<AKAZE> akaze = AKAZE::create();

	akaze->detectAndCompute(imageLeft, noArray(), keyPointsLeft, descriptorsLeft);
	akaze->detectAndCompute(imageRight, noArray(), KeyPointsRigth, descriptorsRigth);

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

	// write image to disk
	string imageMatchesName("AKAZEmatches.jpg");
	fileFound = getPathForThisFile(imageMatchesName,pathToFile);
	if (fileFound){
		boost::filesystem::remove(pathToFile);
	}
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
	F_Matrix = findFundamentalMat(leftPoints,rightPoints,FM_RANSAC,3,0.999);
	F_Matrix.copyTo(F_MatrixExtern);

	// print the matrix
	string F_Name("Fundamental Matrix");
	printMatrix(F_Matrix, F_Name);

	// probe F
	cout << "det(F): " << cv::determinant(F_Matrix) << '\n' << endl;
}


// find the esential matrix
void StereoCamera::findEssentialMatrix(cv::Mat &EsentialMatrix) {
	
	cv::Mat tmpResult, normalized_E;
	cv::Mat K_left = Mat::eye(3, 3, CV_64F);
	cv::Mat K_right = Mat::eye(3, 3, CV_64F);

	leftCamera.getIntrinsicMatrix(K_left);
	rightCamera.getIntrinsicMatrix(K_right);

	// print internal matrices
	string KL_Name("kLeft Matrix");
	printMatrix(K_left, KL_Name);

	string KR_Name("kRight Matrix");
	printMatrix(K_right, KR_Name);

	gemm(K_right,F_Matrix,1.0,noArray(),0.0,tmpResult,GEMM_1_T);
	gemm(tmpResult,K_left,1.0,noArray(),0.0,E_Matrix);
	E_Matrix.copyTo(EsentialMatrix);
	
	// print the matrix
	string E_Name("Essential Matrix");
	printMatrix(E_Matrix, E_Name);

	cv::Mat w, u, vt;
	cv::SVD::compute(E_Matrix,w,u,vt);

	printMatrix(w,string("w"));
	printMatrix(w, string("u"));
	printMatrix(w, string("vt"));

	cv::normalize(E_Matrix, normalized_E, E_Matrix.at<double>(2, 2));
	string ENorm_Name("Essential Matrix Normalized");
	printMatrix(normalized_E, ENorm_Name);

	cv::SVD::compute(normalized_E, w, u, vt);
	printMatrix(w, string("w"));
	printMatrix(w, string("u"));
	printMatrix(w, string("vt"));

	normalized_E.copyTo(E_Matrix);

	// find the matrix E directly
	vector<cv::Point2f> leftPoints, rightPoints, leftPointsNormalized, rightPointsNormalized;
	KeyPoint::convert(matchesLeft, leftPoints);
	KeyPoint::convert(matchesRight, rightPoints);

	// get an average value for the focal lengths and principal points
	double focalLength = (CameraUsefulParametersLeft.focalLength + CameraUsefulParametersRight.focalLength) / 2;
	cv::Point2d principalPoint((CameraUsefulParametersLeft.principalPointX + CameraUsefulParametersRight.principalPointX) / 2,
		(CameraUsefulParametersLeft.principalPointY + CameraUsefulParametersRight.principalPointY) / 2);

	// save these values to further calculus
	averageFocalLength = focalLength;
	averagePrincipalPoint = principalPoint;

	cv::Mat E_openCV = findEssentialMat(leftPoints,rightPoints,focalLength,principalPoint,RANSAC,0.999);
	E_openCV.copyTo(EsentialMatrix);
	E_openCV.copyTo(E_Matrix);
	
	string ECV_Name("Essential Matrix OpenCV");
	printMatrix(E_openCV, ECV_Name);

	cv::SVD::compute(E_openCV, w, u, vt);

	printMatrix(w, string("w"));
	printMatrix(w, string("u"));
	printMatrix(w, string("vt"));

	// get E using normalized points xnorm= Kinv*x
	vector<cv::Point2f> leftNormalPoints, rightNormalPoints;
	vector<cv::Point3f> leftNormalizedPoints, rightNormalizedPoints;
	normalizePoints(K_left, leftPoints, leftNormalizedPoints);
	normalizePoints(K_right, rightPoints, rightNormalizedPoints);

	convertPointsFromHomogeneous(leftNormalizedPoints, leftNormalPoints);
	convertPointsFromHomogeneous(rightNormalizedPoints, rightNormalPoints);

	cv::Mat E_norm = findEssentialMat(leftNormalPoints, rightNormalPoints, focalLength, principalPoint, RANSAC, 0.999);
	E_norm.copyTo(EsentialMatrix);
	E_norm.copyTo(E_Matrix);

	string Enorm_Name("Essential Matrix using normalized points x_norm = Kinverted*x");
	printMatrix(E_norm, Enorm_Name);

	cv::SVD::compute(E_norm, w, u, vt);

	// check SVD values, 2 have to be equal and the last must be zero
	printMatrix(w, string("w"));
	//printMatrix(u, string("u"));
	//printMatrix(vt, string("vt"));

	cout << "Det(E)= " << cv::determinant(E_norm) << "\n" << endl;

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

	// print the matrix
	string R_Name("Rotation Matrix");
	printMatrix(rotationMatrix, R_Name);

	// angles in degrees
	cout << "Alpha degrees "<< (Alpha * 180) / CV_PI << '\n' << endl;
	cout << "Beta degrees " << (Beta * 180) / CV_PI  << '\n' << endl;
	cout << "Gamma degrees " << (Gamma * 180) / CV_PI << '\n' << endl;
}

// get the traslation bewteen two cameras
void StereoCamera::getTraslationFromMatrix(cv::Mat &TraslationMatrix, double &X_shift, double &Y_shift, double &Z_shift){

	X_shift = TraslationMatrix.at<double>(0,0);
	Y_shift = TraslationMatrix.at<double>(1,0);
	Z_shift = TraslationMatrix.at<double>(2,0);

	// print the matrix
	string T_Name("traslation Matrix");
	printMatrix(TraslationMatrix, T_Name);
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

	// get the traslation
	double X_shift, Y_shift, Z_shift;
	getTraslationFromMatrix(traslation,X_shift, Y_shift, Z_shift);
}

// get a pair of correct matrices P,P' from the essential matrix
void StereoCamera::findProjectionMatricesFrom_E_Matrix(vector<cv::Mat> &ProjectionMatrices){

	// get the K matrices
	cv::Mat K_left = Mat::eye(3, 3, CV_64F);
	cv::Mat K_right = Mat::eye(3, 3, CV_64F);

	// build the real P,P' matrices to perform which is the good one
	cv::Mat P_realLeft(3, 4, CV_64F);
	cv::Mat P_real_recovered(3, 4, CV_64F);
	cv::Mat P_recovered(3, 4, CV_64F);

	// Find the correct P matrix
	cv::Mat triangulatedPointsRecovered;
	vector<cv::Point2f> leftPoints, rightPoints;

	// Get the matched points
	KeyPoint::convert(matchesLeft, leftPoints);
	KeyPoint::convert(matchesRight, rightPoints);

	vector<cv::Point2f> leftNormalPoints, rightNormalPoints;
	vector<cv::Point3f> leftNormalizedPoints, rightNormalizedPoints;
	leftCamera.getIntrinsicMatrix(K_left);
	rightCamera.getIntrinsicMatrix(K_right);

	normalizePoints(K_left, leftPoints, leftNormalizedPoints);
	normalizePoints(K_right, rightPoints, rightNormalizedPoints);
	convertPointsFromHomogeneous(leftNormalizedPoints, leftNormalPoints);
	convertPointsFromHomogeneous(rightNormalizedPoints, rightNormalPoints);

	// refine the matches
	vector<cv::Point2f> refinedMatchesLeft, refinedMatchesRight;
	correctMatches(F_Matrix, leftPoints, rightPoints, refinedMatchesLeft, refinedMatchesRight);

	// build the normalized projection Cameras
	// make the first projection matrix P = [I 0]
	PLeft = Mat::eye(3, 4, CV_64F);
	ProjectionMatrices.push_back(PLeft);

	// Find the R and t from E matrix using recoverPose function
	// build the alternatives for the second projection matrix 
	// according to section 9.6 from Zisserman book on 2nd edition
	// it is implemented at the recoverPose OpenCV function
	cv::Mat R_recovered, t_recovered, mask;

	recoverPose(E_Matrix, leftNormalPoints, rightNormalPoints, R_recovered, t_recovered,
		averageFocalLength,averagePrincipalPoint,mask);

	// build the projection matrices options
	build_Projection_Matrix(P_recovered, R_recovered,t_recovered);

	string testPRight("P_Recovered");
	printMatrix(P_recovered, testPRight);	

	// save the right projection Matrix
	ProjectionMatrices.push_back(P_recovered);
	P_real_recovered.copyTo(PRight);
	cout << " P_real_recovered choosed" << '\n' << endl;

	// test points depth
	vector<cv::Point3f> testTriangulatedPoints;
	triangulatePoints(PLeft, P_recovered, leftNormalPoints, rightNormalPoints, triangulatedPointsRecovered);

	// convert from homogeneous
	cv::convertPointsFromHomogeneous(triangulatedPointsRecovered.reshape(4, 1), testTriangulatedPoints);

}


// find  a 3d point position
void StereoCamera::find3DPoint() {
	// TODO - implement StereoCamera::find3DPoint
	throw "Not yet implemented";
}

// build a normalized projection Matrix P = [R t]
void StereoCamera::build_Projection_Matrix(cv::Mat &P, cv::Mat R, cv::Mat T){

	// get the size of the matrix
	cv::Size matrixSize = P.size();

	// fill the Rotation Part
	for (int i = 0; i < matrixSize.height;i++){
		for (int j = 0; j < matrixSize.width - 1;j++){

			P.at<double>(i, j) = R.at<double>(i,j);
		}
	}

	// fill the Traslation Part
	for (int i = 0; i < matrixSize.height;i++){
		
		P.at<double>(i, matrixSize.width - 1) = T.at<double>(i,0);
	}
	
}

// test a 3D point: all the values must be positive to be correct
//					positive because are in the front of the camera 
//					for details see Zisserman book  on multiple view geometry section 9.6
bool StereoCamera::test3DPoint(vector<cv::Point3f> pointsToTest){

	bool isCorrect = false;
	bool xValue, yValue, zValue;

	for (int i = 0; i < pointsToTest.size(); i++){
	
		zValue = std::signbit(pointsToTest.at(i).z);

		if (!zValue){
			isCorrect = true;
		}
		else{
			isCorrect = false;
			break;
		}
	
	}	
	return isCorrect;
}

// normalize points with a K matrix x_norm = Kinverted*x
void StereoCamera::normalizePoints(cv::Mat K, vector<cv::Point2f> &inputPoints, vector<cv::Point3f> &normalizedPoints){

	cv::Mat tmpMatrix,tmpMatrix2, K_inverted;
	cv::Point3f currentNormalizedPoint;
	cv::Point3f currentNormPoint;
	vector<cv::Point3f> tmpNormalizedPoints;
	convertPointsToHomogeneous(inputPoints, tmpNormalizedPoints);
	tmpMatrix = cv::Mat(3,1,CV_64F);
	K_inverted = K.inv(DECOMP_SVD);

	int pointsSize = inputPoints.size();
	for (int i = 0; i < pointsSize; i++){		

		tmpMatrix.at<double>(0,0) = tmpNormalizedPoints.at(i).x;
		tmpMatrix.at<double>(1,0) = tmpNormalizedPoints.at(i).y;
		tmpMatrix.at<double>(2,0) = tmpNormalizedPoints.at(i).z;

		gemm(K_inverted, tmpMatrix, 1.0, cv::noArray(), 0.0, tmpMatrix2);
		currentNormalizedPoint.x = tmpMatrix2.at<double>(0,0);
		currentNormalizedPoint.y = tmpMatrix2.at<double>(1,0);
		currentNormalizedPoint.z = tmpMatrix2.at<double>(2,0);
		normalizedPoints.push_back(currentNormalizedPoint);

		// save the normalized point
		currentNormPoint.x = currentNormalizedPoint.x;
		currentNormPoint.y = currentNormalizedPoint.y;
		currentNormPoint.z = currentNormalizedPoint.z;
		
	}
}

// perform a tracking test for verification of
//			F,E,P and P' matrices
void StereoCamera::trackTestPointer(){

	int cameraID_Left, cameraID_Right;
	string cameraName_Left("LeftCamera"), cameraName_Right("RightCamera");

	cv::Mat DistortionCoeffsLeft = Mat::zeros(8, 1, CV_64F);
	cv::Mat DistortionCoeffsRight = Mat::zeros(8, 1, CV_64F);

	cv::Mat K_left = Mat::eye(3, 3, CV_64F);
	cv::Mat K_right = Mat::eye(3, 3, CV_64F);

	// get the matrix K and distortion coefficients
	leftCamera.getIntrinsicMatrix(K_left);
	leftCamera.getDistortionMatrix(DistortionCoeffsLeft);

	rightCamera.getIntrinsicMatrix(K_right);
	rightCamera.getDistortionMatrix(DistortionCoeffsRight);

	// get the corresponding input device IDs
	leftCamera.getCameraID(cameraID_Left);
	rightCamera.getCameraID(cameraID_Right);

	// start tracking
	TrackerPoint trackerL(cameraID_Left, cameraName_Left, K_left, DistortionCoeffsLeft);
	TrackerPoint trackerR(cameraID_Right,cameraName_Right,K_right, DistortionCoeffsRight);	

	// register(connect) the signals to the slots(receivers) functions
	trackerL.registerSignal(boost::bind(&StereoCamera::getLeftPoint, this, _1));
	trackerR.registerSignal(boost::bind(&StereoCamera::getRightPoint,this, _1));

	// register(connect) the signal to results evaluation
	trackerL.registerEvaluateSignal(boost::bind(&StereoCamera::evaluateResults, this));		

	// create threads for reading data
	std::function<void(TrackerPoint)> threadTrackingFunction = &TrackerPoint::startTracking;
	
	std::thread leftTrackerThread(threadTrackingFunction,trackerL);
	std::thread rightTrackerThread(threadTrackingFunction,trackerR);

	leftTrackerThread.join();
	rightTrackerThread.join();	

}

// print the contest of a given Matrix
void StereoCamera::printMatrix(cv::Mat Matrix, string matrixName){

	// get the size of the matrix
	string rowContent, rowItem;

	cv::Size matrixSize = Matrix.size();

	cout << matrixName << " content is: " << '\n' << endl;
	for (int i = 0; i < matrixSize.height;i++){
		for (int j = 0; j < matrixSize.width;j++){

			rowItem.assign(std::to_string(Matrix.at<double>(i, j)));
			rowContent.append(rowItem + string("  "));
		}
		cout << rowContent << '\n' << endl;
		rowContent.clear();
	}

}

// get the left point from tracking test
void StereoCamera::getLeftPoint(cv::Point2f leftPoint){

	// update current point position
	leftTrackedPoint = leftPoint;
	cout << "Left Position is: " << leftPoint.x << " " << leftPoint.y << "\n" << endl;
}

// get the right point from tracking test
void StereoCamera::getRightPoint(cv::Point2f rightPoint){

	// update current point position
	rightTrackedPoint = rightPoint;
	cout << "Right Position is: " << rightPoint.x << " " << rightPoint.y << "\n" << endl;

}

// evaluate results from calibration
void StereoCamera::evaluateResults(void){

	// get the K matrices
	cv::Mat K_left = Mat::eye(3, 3, CV_64F);
	cv::Mat K_right = Mat::eye(3, 3, CV_64F);
	cv::Mat P_realLeft(3,4, CV_64F);
	cv::Mat P_realRight(3,4, CV_64F);
	
	vector<cv::Point3f> inputLeft, inputRight;

	// take the found Projection matrices P,P' and triangulate the current point
	cv::Mat triangulateTrackedPoint_3D;
	vector<cv::Point2f> leftPoint, rightPoint, leftTestPoint,rightTestPoint;
	vector<cv::Point3f> leftNormalizedPoint, rightNormalizedPoint;

	leftPoint.push_back(leftTrackedPoint);
	rightPoint.push_back(rightTrackedPoint);

	normalizePoints(K_left, leftPoint, leftNormalizedPoint);
	normalizePoints(K_right, rightPoint, rightNormalizedPoint);

	convertPointsFromHomogeneous(leftNormalizedPoint, leftTestPoint);
	convertPointsFromHomogeneous(rightNormalizedPoint, rightTestPoint);

	// get the K,k' matrices
	leftCamera.getIntrinsicMatrix(K_left);
	rightCamera.getIntrinsicMatrix(K_right);	

	// build the matrices P =K*PNormalized
	//                    P' = K'*P'Normalized	
	gemm(K_left,PLeft,1.0,cv::noArray(),0.0,P_realLeft);
	gemm(K_right,PRight,1.0, cv::noArray(), 0.0, P_realRight);

	triangulatePoints(PLeft, PRight, leftTestPoint, rightTestPoint, triangulateTrackedPoint_3D);

	// converts from homogeneous
	vector<cv::Point3f> trackedPoint_3D;
	cv::convertPointsFromHomogeneous(triangulateTrackedPoint_3D.reshape(4,1), trackedPoint_3D);
	
	// prints X,Y,Z values
	cout << "the current position of tracked point is: \n"
		<< "X " << trackedPoint_3D.front().x << "\n"
		<< "Y " << trackedPoint_3D.front().y << "\n"
		<< "Z " << trackedPoint_3D.front().z << "\n" << endl;

	// convert to homogeneous points
	cv::convertPointsToHomogeneous(leftPoint, inputLeft);
	cv::convertPointsToHomogeneous(rightPoint, inputRight);

	// call linear LS triangulation
	vector<cv::Point3f> point3D;
	linearLSTriangulation(leftNormalizedPoint,PLeft,rightNormalizedPoint, PRight,point3D);

	// prints X,Y,Z values
	cout << "the current position of tracked point using linear LS triangulation is: \n"
		<< "X " << point3D.front().x << "\n"
		<< "Y " << point3D.front().y << "\n"
		<< "Z " << point3D.front().z << "\n" << endl;

		
}

// perform a linear triangulation
// from More that technical web blog
void StereoCamera::linearLSTriangulation(vector<cv::Point3f> PointLeft, cv::Mat P1, vector<cv::Point3f> PointRight, cv::Mat P2, vector<cv::Point3f> &triangulatedPoint){

	// build matrix A for homogeneous equation system Ax = 0
	// it assumes X = (x,y,z,1), for linear LS method
	// which turns it into AX = B system,
	// where A is a 4x3, X is 3x1 and B is 4x1

	cv::Point3f Point1 = PointLeft.front();
	cv::Point3f Point2 = PointRight.front();
	cv::Point3f point3D;

	int x1 = Point1.x;int y1 = Point1.y;
	int x2 = Point2.x;int y2 = Point2.y;

	cv::Mat A =(Mat_<double>(4,3) << x1*P1.at<double>(2, 0) - P1.at<double>(0, 0), x1*P1.at<double>(2, 1) - P1.at<double>(0, 1), x1*P1.at<double>(2, 2) - P1.at<double>(0, 2),
									 y1*P1.at<double>(2, 0) - P1.at<double>(1, 0), y1*P1.at<double>(2, 1) - P1.at<double>(1, 1), y1*P1.at<double>(2, 2) - P1.at<double>(1, 2),
									 x2*P2.at<double>(2, 0) - P2.at<double>(0, 0), x2*P2.at<double>(2, 1) - P2.at<double>(0, 1), x2*P2.at<double>(2, 2) - P2.at<double>(0, 2),
									 y2*P2.at<double>(2, 0) - P2.at<double>(1, 0), y2*P2.at<double>(2, 1) - P2.at<double>(1, 1), y2*P2.at<double>(2, 2) - P2.at<double>(1, 2));


	cv::Mat B = (Mat_<double>(4, 1) << -x1*P1.at<double>(2, 3) - P1.at<double>(0, 3),
									   -y1*P1.at<double>(2, 3) - P1.at<double>(1, 3), 
							           -x2*P2.at<double>(2, 3) - P2.at<double>(0, 3), 
							           -y2*P2.at<double>(2, 3) - P2.at<double>(1, 3));

	cv::Mat X;
	solve(A, B, X, cv::DECOMP_SVD);

	// return the results
	point3D.x = X.at<double>(0,0);
	point3D.y = X.at<double>(1,0);
	point3D.z = X.at<double>(2,0);

	triangulatedPoint.push_back(point3D);
	
}











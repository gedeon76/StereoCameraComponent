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
	bool fileFound;
	string pathToFile;
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
	F_Matrix = findFundamentalMat(leftPoints,rightPoints,FM_RANSAC,3,0.99);
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

	cv:Mat w, u, vt;
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

	cv::Mat E_openCV = findEssentialMat(leftPoints,rightPoints,focalLength,principalPoint,RANSAC,0.999);
	E_openCV.copyTo(EsentialMatrix);
	E_openCV.copyTo(E_Matrix);
	
	string ECV_Name("Essential Matrix OpenCV");
	printMatrix(E_openCV, ECV_Name);

	cv::SVD::compute(E_openCV, w, u, vt);

	printMatrix(w, string("w"));
	printMatrix(w, string("u"));
	printMatrix(w, string("vt"));

	// get E using normalized points

	//vector<cv::Point2f> leftNormalizedPoints, rightNormalizedPoints;
	//normalizePoints(K_left, leftPoints, leftNormalizedPoints);
	//normalizePoints(K_right, rightPoints, rightNormalizedPoints);

	/*cv::Mat E_norm = findEssentialMat(leftNormalizedPoints, rightNormalizedPoints, focalLength, principalPoint, RANSAC, 0, 999);
	E_norm.copyTo(EsentialMatrix);
	E_norm.copyTo(E_Matrix);

	string Enorm_Name("Essential Matrix OpenCV normalized");
	printMatrix(E_norm, Enorm_Name);

	cv::SVD::compute(E_norm, w, u, vt);

	printMatrix(w, string("w"));
	printMatrix(w, string("u"));
	printMatrix(w, string("vt"));*/


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

	// build the normalized projection Cameras
	// make the first projection matrix P = [I 0]
	PLeft = Mat::eye(3, 4, CV_64F);

	// built the alternatives for the second projection matrix 
	// according to section 9.6 from Zisserman book on 2nd edition
	cv::Mat P1(3, 4, CV_64F), P2(3, 4, CV_64F), P3(3, 4, CV_64F), P4(3, 4, CV_64F);
	cv::Mat R1, R2, t;

	decomposeEssentialMat(E_Matrix, R1, R2, t);

	// P1
	build_Projection_Matrix(P1, R1, t);
	build_Projection_Matrix(P2, R1, -t);
	build_Projection_Matrix(P3, R2, t);
	build_Projection_Matrix(P4, R2, -t);

	string testP1("P1");
	printMatrix(P1, testP1);

	string testP2("P2");
	printMatrix(P2, testP2);

	string testP3("P3");
	printMatrix(P3, testP3);

	string testP4("P4");
	printMatrix(P4, testP4);

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

// normalize points with a K matrix
void StereoCamera::normalizePoints(cv::Mat K, vector<cv::Point2f> &inputPoints, vector<cv::Point2f> &normalizedPoints){

	cv::Mat tmpMatrix,tmpMatrix2;
	cv::Point2f currentNormalizedPoint;
	vector<cv::Point3f> tmpNormalizedPoints;
	convertPointsToHomogeneous(inputPoints, tmpNormalizedPoints);
	tmpMatrix = cv::Mat(tmpNormalizedPoints);
	
	int pointsSize = inputPoints.size();
	for (int i = 0; i < pointsSize; i++){		

		//tmpMatrix.at<double>(0, 0) = tmpNormalizedPoints.at(i).x;
		//tmpMatrix.at<double>(1, 0) = tmpNormalizedPoints.at(i).y;
		//tmpMatrix.at<double>(2, 0) = tmpNormalizedPoints.at(i).z;

		gemm(K.inv(DECOMP_SVD), tmpMatrix, 1.0, cv::noArray(), 0.0, tmpMatrix2);
		currentNormalizedPoint.x = tmpMatrix2.at<double>(0,0);
		currentNormalizedPoint.y = tmpMatrix2.at<double>(1,0);
		normalizedPoints.push_back(currentNormalizedPoint);
		
	}
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










#include "CameraCalibration.h"

CameraCalibration::CameraCalibration() 
{
	clock_t prevTimestamp = 0;
	intrinsicK_Matrix = Mat::eye(3, 3, CV_64F);
	distortionCoefficients = Mat::zeros(8, 1, CV_64F);	

	// initialize the 3D position for all asymetric circles
	circle_Mis_Positions = {M1,M2,M3,M4,M5,M6,M7,M8,M9,M10,M11,
							M12,M13,M14,M15,M16,M17,M18,M19,M20,M21,M22,
							M23,M24,M25,M26,M27,M28,M29,M30,M31,M32,M33,
							M34,M35,M36,M37,M38,M39,M40,M41,M42,M43,M44};
	
}


CameraCalibration::~CameraCalibration()
{

}


/*
	The next static functions are defined to serialize the xml files according
	to OpenCV documentation under the title:

	File Input and Output using XML and YAML files
*/

/// static function that reads the settings file
/// @param[in] node FileNode to read from
/// @param[out] Settings Settings to be saved from FileNode
/// @param[in] default_value value by default if FileNode is empty
static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
	if (node.empty()){
		x = default_value;
	}
	else{
		x.read(node);
	}
}

/// static function that reads the results file
/// @param[in] node FileNode to read from
/// @param[out] Results result to be saved from FileNode
/// @param[in] default_value value by default if FileNode is empty
static void read(const FileNode& node, Results& result,const Results& default_value = Results())
{
	if (node.empty())
	{
		result = default_value;
	} else
	{
		result.read(node);
	}
}

/// static function that writes the results file
static  void write(FileStorage& fs, const string&,const Results& result)
{
	result.write(fs);
}


int CameraCalibration::readSettings(string &inputSettingsFile)
{

	FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
		return -1;
	}

	fs["Settings"] >> s;
	fs.release();                                         // close Settings file

	if (!s.goodInput)
	{
		cout << "Invalid input detected. Application stopping. " << endl;
		return -1;
	}

	
}

int CameraCalibration::readResults(string &outputResultsFile) 
{

	FileStorage fs(outputResultsFile, FileStorage::READ);	// read the results file
	if (!fs.isOpened())
	{
		cout << "Could not open the results file \"" << outputResultsFile << "\"" << endl;
		return -1;
	}

	fs["Results"] >> calibrationResults;	
	fs.release();
	

}

/// it has multithread support under c++ 11
void CameraCalibration::getImagesAndFindPatterns(const string &cameraName)
{
	// set mode
	mode = CAPTURING;
	frameCounter = 0;
	imageCounter = 0;
	capturedFrame currentImage;

	// read current path
	currentPath = boost::filesystem::current_path();
	resultsPath = currentPath;
	cout << "current Results Path" << currentPath << '\n' << endl;	
	
	//mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;// check enum type
	
	// Capture only the frames settled in the configuration file 
	// in the original code it was a endless loop
	for (int i = 0;; ++i)
	{

		Mat view,currentView;
		bool blinkOutput = false;
		
		// capture the image
		view = s.nextImage();
		frameCounter = frameCounter + 1;
		
		
		//------------------------- Show original distorted image -----------------------------------

		Mat originalView = view.clone();
		//imshow("original Image", originalView);

		//-----  If no more image, or got enough, then stop calibration and show result -------------
		if (mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames)
		{
			if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints))
				mode = CALIBRATED;
			else
				mode = DETECTION;
		}
		if (view.empty())          // If no more images then run calibration, save and stop loop.
		{
			if (imagePoints.size() > 0)
				runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints);
			break;
		}

		
		imageSize = view.size();  // Format input image.
		if (s.flipVertical)    flip(view, view, 0);

		vector<Point2f> pointBuf;

		bool found;
		switch (s.calibrationPattern) // Find feature points on the input format
		{
		case Settings::CHESSBOARD:
			found = findChessboardCorners(view, s.boardSize, pointBuf,
				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
			break;
		case Settings::CIRCLES_GRID:
			found = findCirclesGrid(view, s.boardSize, pointBuf);
			break;
		case Settings::ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid(view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
			break;
		default:
			found = false;
			break;
		}

		if (found)                // If done with success,
		{
			// improve the found corners' coordinate accuracy for chessboard
			if (s.calibrationPattern == Settings::CHESSBOARD)
			{
				Mat viewGray;
				cvtColor(view, viewGray, COLOR_BGR2GRAY);
				cornerSubPix(viewGray, pointBuf, Size(11, 11),
					Size(-1, -1), TermCriteria(TermCriteria::EPS+TermCriteria::MAX_ITER, 30, 0.1));
			}

			if (mode == CAPTURING &&  // For camera only take new samples after delay time
				(!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC))
			{
				imagePoints.push_back(pointBuf);
				prevTimestamp = clock();
				blinkOutput = s.inputCapture.isOpened();
				circlePoints = pointBuf;	// save ordered circle points
			}

			// Draw the corners with ID according order criteria distance(x+y) from origin point 1
			savedImage = view.clone();
			drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);

			// order the points according to d(x+y) from the upper left corner that is used as the origin frame
			std::sort(pointBuf.begin(), pointBuf.end(), [](const cv::Point2f &a, const cv::Point2f &b)
						{return ((a.x + a.y) < (b.x + b.y)); });
			
			int pointCounter = 1;
			for (auto k:pointBuf){
						
				cv::putText(view,std::to_string(pointCounter),cv::Point(k.x,k.y),cv::FONT_HERSHEY_PLAIN,1.0,cv::Scalar(255,0,0),1);
				pointCounter = pointCounter + 1;
			}			
		}

		//----------------------------- Output Text ------------------------------------------------		

		string msg = (mode == CAPTURING) ? "100/100" :
			mode == CALIBRATED ? "Calibrated" : "the images are being captured";
		int baseLine = 0;
		Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
		Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

		if (mode == CAPTURING)
		{
			if (s.showUndistorsed)
				msg = format("%d/%d Undist", (int)imagePoints.size(), s.nrFrames);
			else
				msg = format("%d/%d", (int)imagePoints.size(), s.nrFrames);
		}

		putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

		if (blinkOutput){

			bitwise_not(view, view);

			// save the image used for calibration to disk
			imageCounter = imageCounter + 1;
			string pathToFile;
			string filename;
			if (imageCounter <= s.nrFrames)
			{

				string imageName{ "Image" + string(std::to_string(imageCounter)) + cameraName + ".jpg" };
				boost::filesystem::path p{ "/" };	// add a slash to generate a portable string
				filename.assign(resultsPath.string() + p.generic_string() + imageName);

				vector<int> compression_params;
				compression_params.push_back(IMWRITE_JPEG_QUALITY);
				compression_params.push_back(100);
				
				// check if the file already exist? if yes, erase it
				bool found = getPathForThisFile(imageName,pathToFile);
				if (found){
					boost::filesystem::remove(pathToFile);
				}
				// write the new version of the file
				cv::imwrite(filename, savedImage);
				
				// save the points use to estimate the scale factor
				int currentBufferPosition = imagePoints.size();

				int pointCounter = 1;
				circlePatternInfo currentCircle;
				circlesDataPerImage dataCurrentImage;
				vector<circlePatternInfo> circlesInfoFromThisImage;
				for (auto k : circlePoints){

					currentCircle.circleID = pointCounter;
					currentCircle.circlePosition = cv::Point2f(k.x, k.y);
					currentCircle.circle3DPosition = circle_Mis_Positions.at(pointCounter - 1);
					circlesInfoFromThisImage.push_back(currentCircle);
					pointCounter = pointCounter + 1;
				}
				circlePoints.clear();
				dataCurrentImage.imageID= imageCounter;
				dataCurrentImage.cameraID = s.cameraID;
				dataCurrentImage.circlesData = circlesInfoFromThisImage;

				// save all the data from the asymetric circles pattern
				DataFromCirclesPattern.push_back(dataCurrentImage);
				
			}
		}
		
		//------------------------- Video capture  output  undistorted ------------------------------
		if (mode == CALIBRATED && s.showUndistorsed)
		{
			Mat temp = view.clone();
			undistort(temp, view, cameraMatrix, distCoeffs);
			string msgEsckey = "Press 'esc' key to quit";
			putText(view, msgEsckey, textOrigin, 1, 1, GREEN, 2);			

		}

		//------------------------------ Show image and check for input commands -------------------
		imshow(cameraName, view);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		cv::waitKey(30);

		char c = waitKey(1);

		if (c == ESC_KEY)    //  Escape key
			break;			// Breaks the capture loop

	}

}


double CameraCalibration::computeReprojectionErrors(vector<vector<Point3f>>& objectPoints,
							vector<vector<Point2f>>& imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs, 
							Mat& cameraMatrix, Mat& distCoeffs, vector<float>& perViewErrors)
{
	
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

void CameraCalibration::calcBoardCornerPositions(Size boardSize, float squareSize, 
										vector<Point3f>& corners, Settings::Pattern patternType)
{
	
	corners.clear();

	switch (patternType)
	{
	case Settings::CHESSBOARD:
	case Settings::CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				corners.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
		break;

	case Settings::ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
		break;
	default:
		break;
	}
}

bool CameraCalibration::runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix, 
								Mat& distCoeffs, vector<vector<Point2f>> imagePoints, vector<Mat>& rvecs,
								vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr) 
{
	
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (s.flag & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = 1.0;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, s.flag | CALIB_FIX_K4 | CALIB_FIX_K5);

	cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok; 
}

void CameraCalibration::saveCameraParams(Settings& s, Size& imageSize, Mat& cameraMatrix, 
									Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs, 
									vector<float>& reprojErrs, vector<vector<Point2f>>& imagePoints,
									double totalAvgErr) 
{
	FileStorage fs(s.outputFileName, FileStorage::WRITE);

	time_t tm;
	time(&tm);
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "Results"; 
	fs << "{" <<"calibration_Time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_Width" << imageSize.width;
	fs << "image_Height" << imageSize.height;
	fs << "board_Width" << s.boardSize.width;
	fs << "board_Height" << s.boardSize.height;
	fs << "square_Size" << s.squareSize;

	if (s.flag & CALIB_FIX_ASPECT_RATIO)
		fs << "FixAspectRatio" << s.aspectRatio;

	if (s.flag)
	{
		sprintf_s(buf, "flags: %s%s%s%s",
			s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
			s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
			s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
			s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
		//cvWriteComment(*fs, buf, 0);
		
	}

	fs << "flagValue" << s.flag;

	fs << "Camera_Matrix" << cameraMatrix;
	fs << "Distortion_Coefficients" << distCoeffs;

	fs << "Avg_Reprojection_Error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		//cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
		fs << "Extrinsic_Parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "Image_points" << imagePtMat;//Image_points

	}

	if (!DataFromCirclesPattern.empty()){

		// save the circle pattern positions for this camera and the corresponding calibration images
		// as a vector 1x6 <circleID, (x,y) 2D position,(X,Y,Z) 3D position> for each circle found
		cv::Mat circlesDataMatrix;

		for (int k = 0; k < DataFromCirclesPattern.size(); k++){

			int cameraID = DataFromCirclesPattern.at(k).cameraID;
			int imageID = DataFromCirclesPattern.at(k).imageID;

			cv::Mat circlesData = cv::Mat::zeros(1, 6, CV_64F);
			 
			circlesData.at<double>(0, 0) = cameraID;
			circlesData.at<double>(0, 1) = imageID;
			circlesData.at<double>(0, 2) = 0;
			circlesData.at<double>(0, 3) = 0;
			circlesData.at<double>(0, 4) = 0;
			circlesData.at<double>(0, 5) = 0;

			// save ID header row
			circlesDataMatrix.push_back(circlesData);

			for (int i = 0; i < DataFromCirclesPattern.at(k).circlesData.size(); i++){

				int circleID = DataFromCirclesPattern.at(k).circlesData.at(i).circleID;
				cv::Point2f circlePosition = DataFromCirclesPattern.at(k).circlesData.at(i).circlePosition;
				cv::Point3f circle3DPosition = DataFromCirclesPattern.at(k).circlesData.at(i).circle3DPosition;

				// build the matrix data
				circlesData.at<double>(0, 0) = circleID;
				circlesData.at<double>(0, 1) = circlePosition.x;
				circlesData.at<double>(0, 2) = circlePosition.y;
				circlesData.at<double>(0, 3) = circle3DPosition.x;
				circlesData.at<double>(0, 4) = circle3DPosition.y;
				circlesData.at<double>(0, 5) = circle3DPosition.z;

				circlesDataMatrix.push_back(circlesData);
			}

		}
		Mat finalCirclesDataMatrix(circlesDataMatrix.rows, 6, CV_64F);
		circlesDataMatrix.copyTo(finalCirclesDataMatrix);

		fs << "Circle_Data" << finalCirclesDataMatrix;		
		
	}


		
}

bool CameraCalibration::runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, 
											Mat& distCoeffs, vector<vector<Point2f>> imagePoints) 
{
	
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
		reprojErrs, totalAvgErr);
	cout << (ok ? "Calibration succeeded" : "Calibration failed")
		<< ". avg re projection error = " << totalAvgErr;

	if (ok)
		saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs,
		imagePoints, totalAvgErr);
	return ok;
}

void CameraCalibration::getIntrinsicMatrix(Mat &intrinsicMatrix)
{
	
	intrinsicK_Matrix = calibrationResults.intrinsicCameraMatrix.clone();
	intrinsicK_Matrix.copyTo(intrinsicMatrix);

}

void CameraCalibration::getExtrinsicParameters(Mat &extrincicParameters){

	extrinsicParametersMatrix = calibrationResults.extrinsicParameters.clone();
	extrinsicParametersMatrix.copyTo(extrincicParameters);

}

void CameraCalibration::getCameraUsefulParameters(cameraData &cameraUsefulParameters)
{
	Size imageSize;
	imageSize.width = calibrationResults.imageWidth;
	imageSize.height = calibrationResults.imageHeight;

	// logitech c270 corresponding to 1/4" sensor
	double sensorWidth = calibrationResults.sensorSizeWidth;
	double sensorHeight = calibrationResults.sensorSizeHeight;
	double fov_X, fov_Y, focalLength, aspectRatio;
	Point2d principalPoint;

	Mat intrinsicFound = calibrationResults.intrinsicCameraMatrix.clone();
	calibrationMatrixValues(intrinsicFound, imageSize, 
		sensorWidth, sensorHeight, fov_X, fov_Y, focalLength, principalPoint, aspectRatio);

	cameraUsefulParameters.imageWidth = imageSize.width;
	cameraUsefulParameters.imageHeight = imageSize.height;
	cameraUsefulParameters.sensorWidth = sensorWidth;
	cameraUsefulParameters.sensorHeight = sensorHeight;
	cameraUsefulParameters.fov_X = fov_X;
	cameraUsefulParameters.fov_Y = fov_Y;
	cameraUsefulParameters.focalLength = focalLength;
	cameraUsefulParameters.principalPointX = principalPoint.x;
	cameraUsefulParameters.principalPointY = principalPoint.y;
	cameraUsefulParameters.aspectRatio = aspectRatio;

}

void CameraCalibration::getDistortionMatrix(Mat &distortionCameraParameters)
{
	distortionCoefficients = calibrationResults.distortionCoefficients.clone();
	distortionCoefficients.copyTo(distortionCameraParameters);
	
}


void CameraCalibration::getImagesUsedForCalibration(vector<capturedFrame> &imageList) 
{
	// check captured images		
	
}

void CameraCalibration::getInfoFromCirclePatterns(vector<circlesDataPerImage> &circlesPatternData){

	
	// Read the circles pattern information
	Mat tmpCircleMat = calibrationResults.circleData.clone();
	tmpCircleMat.copyTo(patternInformation);

	circlePatternInfo  oneCircle;
	std::vector<circlePatternInfo> currentCircles;
	circlesDataPerImage currentImageCirclePatterns;
	int cameraID = 0;
	int imageID = 0;
	int idx = 0;

	int dataPerImageSize = patternInformation.rows / (patternCircleNumber + 1);
	int increase = patternCircleNumber + 1;

	for (int k = 0; k < dataPerImageSize; k++){

		int positionIndex = k*increase;
		if (k == 0){
			cameraID = patternInformation.at<double>(k, 0);
			imageID = patternInformation.at<double>(k, 1);
		}
		else{
			cameraID = patternInformation.at<double>(positionIndex, 0);
			imageID = patternInformation.at<double>(positionIndex, 1);
		}

		// Get the data from the 1x6x44 vector matrix
		for (int i = 0; i < patternCircleNumber; i++){
			
			int shift = positionIndex + 1;
			oneCircle.circleID = patternInformation.at<double>(shift + i, 0);
			oneCircle.circlePosition.x = patternInformation.at<double>(shift + i, 1);
			oneCircle.circlePosition.y = patternInformation.at<double>(shift + i, 2);
			oneCircle.circle3DPosition.x = patternInformation.at<double>(shift + i, 3);
			oneCircle.circle3DPosition.y = patternInformation.at<double>(shift + i, 4);
			oneCircle.circle3DPosition.z = patternInformation.at<double>(shift + i, 5);
			currentCircles.push_back(oneCircle);
		}

		currentImageCirclePatterns.cameraID = cameraID;
		currentImageCirclePatterns.imageID = imageID;
		currentImageCirclePatterns.circlesData = currentCircles;
		currentCircles.clear();

		// save the data to the specific vector
		DataFromCirclesPattern.push_back(currentImageCirclePatterns);
	}

	// save the final data
	circlesPatternData = DataFromCirclesPattern;
	
}

int CameraCalibration::getHowManyImagesWereUsedperCamera()
{
	int imagesNumber;
	imagesNumber = s.nrFrames;
	return imagesNumber;
}

bool CameraCalibration::getPathForThisFile(string &fileName, string &pathFound)
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

	// built the directory to search 2 levels up from current directory
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

void CameraCalibration::getPathForResults(string &pathToResults){

	// recover the path being used to save the xml results file
	pathToResults = resultsPath.generic_string();
}

void CameraCalibration::getCameraID(int &cameraID){

	cameraID = s.cameraID;
}
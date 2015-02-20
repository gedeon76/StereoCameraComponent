#include "Results.h"


Results::Results()
{
	intrinsicCameraMatrix = Mat::eye(3,3,CV_64F);
	distortionCoefficients = Mat::zeros(8, 1, CV_64F);
	
}


Results::~Results()
{
}

void Results::write(FileStorage& fs) const //Write serialization for this class
{
	fs  << "{" << "calibration_Time" << calibrationTime
		<< "nrOfFrames" << nrFrames
		<< "image_Width" << imageWidth
		<< "image_Height" << imageHeight
		<< "board_Width" << boardWidth
		<< "board_Height" << boardHeight
		<< "square_Size" << squareSize
		<< "flagValue" << flagValue

		<< "Camera_Matrix" << intrinsicCameraMatrix
		<< "Distortion_Coefficients" << distortionCoefficients

		<< "Avg_Reprojection_Error" << avgReprojectionError
		<< "Per_View_Reprojection_Errors" << perViewReprojectionErrors

		<< "Extrinsic_Parameters" << extrinsicParameters
		<< "Image_points" << imagePoints
		<< "Circle_Data" << circleData
		<< "}";
}

void Results::read(const FileNode& node)	//Read serialization for this class
{
	node["calibration_Time"] >> calibrationTime;
	node["nrOfFrames"] >> nrFrames;
	node["image_Width"] >> imageWidth;
	node["image_Height"] >> imageHeight;
	node["board_Width"] >> boardWidth;
	node["board_Height"] >> boardHeight;
	node["square_Size"] >> squareSize;
	node["flagValue"] >> flagValue;

	node["Camera_Matrix"] >> intrinsicCameraMatrix;
	node["Distortion_Coefficients"] >> distortionCoefficients;

	node["Avg_Reprojection_Error"] >> avgReprojectionError;
	node["Per_View_Reprojection_Errors"] >> perViewReprojectionErrors;

	node["Extrinsic_Parameters"] >> extrinsicParameters;	
	node["Image_points"] >> imagePoints;

	node["Circle_Data"] >> circleData;

	
}

void Results::interpretate()
{

}
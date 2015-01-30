/*! \brief
	
	This header contain common definitions used in the 
	StereoComponent

*/

#pragma once	

#include <chrono>

// camera identities
enum cameraIdentity{ LEFT_CAMERA, RIGHT_CAMERA };

/// constants for matching
const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
const float match_ratio = 0.5f;		 // Nearest neighbor matching ratio

/// this structure save a captured image used for calibration
struct capturedFrame
{
	cv::Mat image;
	std::chrono::time_point<std::chrono::system_clock> timeofCapture;
};


/// This data structure contains the parameters found for this camera
struct cameraUsefulData
{
	double imageWidth;
	double imageHeight;
	double sensorWidth;
	double sensorHeight;
	double fov_X;
	double fov_Y;
	double focalLength;
	double principalPointX;
	double principalPointY;
	double aspectRatio;
};

/// This structure is used for save sorted matches
struct sortMatch
{
	int originalIndex;			//!< original index for this match
	int currentIndex;			//!< current index in the sorting
	double outliernessMeasure;	//!< diagonal Hat Matrix value for this match
	cv::Point3f  leftPoint;
	cv::Point3f rightPoint;

};

/// This structure is used to save Essential matrices
struct Essential_MatrixData{

	int matrixIndex;
	bool isGoodMatrix;
	double thresholdForResiduals;
	double thresholdForE_Variance;
	double EvarianceValidResiduals;
	double residualsMedian;	
	std::vector<double> residuals;
	std::vector<bool> validResiduals;
	cv::Mat E_Matrix;
};

/// This structure save the final matrices that pass the filtering according to Millnert'06
struct E_MatricesCandidates{

	double qE;
	cv::Mat E_matrix;

};

typedef cameraUsefulData cameraData;
typedef std::vector<cameraData> cameraParameters;
typedef sortMatch sortedMatches;
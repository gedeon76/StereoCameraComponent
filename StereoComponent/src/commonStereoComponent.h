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

typedef cameraUsefulData cameraData;
typedef std::vector<cameraData> cameraParameters;
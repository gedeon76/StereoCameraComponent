/*! \brief
	
	This header contain common definitions used in the 
	StereoComponent

*/

#pragma once	

#include <chrono>

/// constants for matching
const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
const float match_ratio = 0.5f;		 // Nearest neighbor matching ratio

/// constants for tracking
const double akaze_thresh = 3e-4;	// AKAZE detection threshold set to locate about 1000 keypoints
const double ransac_thresh = 2.5f;	// RANSAC inlier threshold
const int min_inliers = 100;		// Minimal number of inliers to draw bounding box
const int stats_update_period = 10;	// On-screen statistics are updated every 10 frames

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
typedef vector<cameraData> cameraParameters;
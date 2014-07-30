/*! \brief
	
	This header contain common definitions used in the 
	StereoComponent

*/

#pragma once		

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
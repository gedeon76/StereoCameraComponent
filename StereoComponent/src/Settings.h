/*! \brief
	
	This is the settings class used
	for reading the xml camera calibration  configuration files 
	
*/

#pragma once

// include OpenCV headers
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2\core.hpp>
#include <opencv2\imgproc.hpp>
#include <opencv2\calib3d.hpp>
#include <opencv2\videoio.hpp>
#include <opencv2\highgui.hpp>


#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };


using namespace cv;
using namespace std;

/// read each image name
/// @param[in] filename Name of the image
/// @param[in] l list of names of the images
bool readStringList(const string& filename, vector<string>& l);


class Settings {

public:
	enum Pattern {
		NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
	};
	enum InputType {
		INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST
	};
	Size boardSize;							///< The size of the board -> Number of items by width and height
	Settings::Pattern calibrationPattern;	///< One of the Chessboard, circles, or asymmetric circle pattern
	float squareSize;						///< The size of a square in your defined unit (point, millimeter,etc).			
	int nrFrames;							///< The number of frames to use from the input for calibration 
	float aspectRatio;						///< The aspect ratio
	int delay;								///< In case of a video input, delay between captures
	bool bwritePoints;						
	bool bwriteExtrinsics;					///< Write extrinsic parameters			
	bool calibZeroTangentDist;				///< Assume zero tangential distortion
	bool calibFixPrincipalPoint;			///< Fix the principal point at the center
	bool flipVertical;						///< Flip the captured images around the horizontal axis
	string outputFileName;					///< The name of the file where to write
	bool showUndistorsed;					///< Show undistorted images after calibration
	string input;							// The input ->
	
	int cameraID;
	vector<string> imageList;
	int atImageList;
	VideoCapture inputCapture;
	Settings::InputType inputType;
	bool goodInput;
	int flag;
private:
	string patternToUse;

public:
	Settings();

	/// write the results of the corresponding calibration to a xml file
	/// @param[in,out] fs It contains the setting class parameters
	void write(FileStorage& fs) const;

	/// read the configuration settings xml file 
	/// @param[in] node It contains the readings from the configuration file 
	void read(const FileNode& node);

	/// interpretate the different values from the xml configuration file
	void interpretate();

	/// get the next image to process
	Mat nextImage();

};

/*! \brief
	
	This is the settings class used
	for the camera calibration on OpenCV
	
*/


// include OpenCV headers
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };


using namespace cv;
using namespace std;

class Settings {

public:
	enum Pattern {
		NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
	};
	enum InputType {
		INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST
	};
	Size boardSize;
	Settings::Pattern calibrationPattern;
	float squareSize;
	int nrFrames;
	float aspectRatio;
	int delay;
	bool bwritePoints;
	bool bwriteExtrinsics;
	bool calibZeroTangentDist;
	bool calibFixPrincipalPoint;
	bool flipVertical;
	string outputFileName;
	bool showUndistorsed;
	string input;
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
	void read(FileNode& node);

	/// interpretate the different values from the xml configuration file
	void interpretate();

	/// get the next image to process
	Mat nextImage();


	/// read each image name
	/// @param[in] filename Name of the image
	/// @param[in] l list of names of the images
	static bool readStringList(string& filename, vector<string>& l);


};

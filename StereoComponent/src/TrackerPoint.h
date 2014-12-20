/*! \brief

	This class is used 
	for track a pointer in an image

	It uses color segmentation in the HSV space

	It is based on web code by Kyle Hounslow
	from British Columbia University at 2013

*/

#pragma once

//#include <opencv2\tracking.hpp>			// opencv contrib module
#include <opencv2\features2d.hpp>
#include <opencv2\videoio.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>
#include <iomanip>
#include <vector>
#include <boost/signals2.hpp>

#include "commonStereoComponent.h"

using namespace std;


// Define some constants

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;

typedef boost::signals2::connection Connection;
typedef boost::signals2::signal<void(cv::Point2f)> signalType;
typedef boost::signals2::signal<void()> signalResults;
static signalType ownSignalL;
static signalType ownSignalR;
static signalResults checkSignalResults;

class TrackerPoint
{	
	
public:	
	TrackerPoint(int camera_ID, string cameraName, cv::Mat K_Matrix, cv::Mat DistortionCoeffs);
	~TrackerPoint();

	void createTrackBars();
	void drawObject(int x, int y, cv::Mat &frame);
	void applyMorphologicalFilters(cv::Mat &threshold);
	void trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed);
	void startTracking();
	void doEmit(int x, int y);	// emit the boost signal
	cv::Point2f getCurrentPoint();

	Connection registerSignal(const signalType::slot_type& slot);	
	Connection registerEvaluateSignal(const signalResults::slot_type& slot);
			
private:
	cv::VideoCapture inputCamera;
	int cameraID;
	string thisCameraName;
	cameraIdentity myIdentity;
	//initial min and max HSV filter values.
	//these will be changed using trackbars
	int H_MIN,H_MAX;
	int S_MIN,S_MAX;
	int V_MIN,V_MAX;

	//names that will appear at the top of each window
	string windowName;
	string windowName1;
	string windowName2;
	string windowName3;
	string trackbarWindowName;

	// current position
	cv::Point2f currentPoint;	
	Connection signalConnection;	
	Connection signalResultsConnection;
};

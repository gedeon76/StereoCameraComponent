#include "TrackerPoint.h"

void on_trackbarCallback(int, void*)
{//This function gets called whenever a
	// trackbar position is changed
}

// constructor
TrackerPoint::TrackerPoint(int camera_ID, string cameraName, cv::Mat K_Matrix, cv::Mat DistortionCoeffs){

	//initial min and max HSV filter values.
	//these will be changed using trackbars
	H_MIN = 0;	H_MAX = 256;
	S_MIN = 0;	S_MAX = 256;
	V_MIN = 0;	V_MAX = 256;

	// set the camera identification
	cameraID = camera_ID;	

	// windows names
	windowName.assign("Original Image"+cameraName);
	windowName1.assign("HSV Image"+cameraName);
	windowName2.assign("Thresholded Image"+cameraName);
	windowName3.assign("After Morphological Operations"+cameraName);
	trackbarWindowName.assign("Trackbars"+cameraName);
}

// destructor
TrackerPoint::~TrackerPoint()
{
}

// create trackbars to set up HSV values
void TrackerPoint::createTrackBars(){

	//create window for trackbars
	cv::namedWindow(trackbarWindowName, 0);

	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN", H_MIN);
	sprintf(TrackbarName, "H_MAX", H_MAX);
	sprintf(TrackbarName, "S_MIN", S_MIN);
	sprintf(TrackbarName, "S_MAX", S_MAX);
	sprintf(TrackbarName, "V_MIN", V_MIN);
	sprintf(TrackbarName, "V_MAX", V_MAX);

	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	cv::createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbarCallback);
	cv::createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbarCallback);
	cv::createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbarCallback);
	cv::createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbarCallback);
	cv::createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbarCallback);
	cv::createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbarCallback);
}

// draw the tracked object
void TrackerPoint::drawObject(int x, int y, cv::Mat &frame){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	cv::circle(frame, cv::Point(x, y), 20, cv::Scalar(0, 255, 0), 2);
	if (y - 25>0)
		cv::line(frame, cv::Point(x, y), cv::Point(x, y - 25), cv::Scalar(0, 255, 0), 2);
	else cv::line(frame, cv::Point(x, y), cv::Point(x, 0), cv::Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		cv::line(frame, cv::Point(x, y), cv::Point(x, y + 25), cv::Scalar(0, 255, 0), 2);
	else cv::line(frame, cv::Point(x, y), cv::Point(x, FRAME_HEIGHT), cv::Scalar(0, 255, 0), 2);
	if (x - 25>0)
		cv::line(frame, cv::Point(x, y), cv::Point(x - 25, y), cv::Scalar(0, 255, 0), 2);
	else cv::line(frame, cv::Point(x, y), cv::Point(0, y), cv::Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		cv::line(frame, cv::Point(x, y), cv::Point(x + 25, y), cv::Scalar(0, 255, 0), 2);
	else cv::line(frame, cv::Point(x, y), cv::Point(FRAME_WIDTH, y), cv::Scalar(0, 255, 0), 2);

	cv::putText(frame, std::to_string(x) + "," + std::to_string(y), cv::Point(x, y + 30), 1, 1, cv::Scalar(0, 255, 0), 2);
}

// apply morphological filters
void TrackerPoint::applyMorphologicalFilters(cv::Mat &threshold){
	
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

	//dilate with larger element so make sure object is nicely visible
	cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));

	cv::erode(threshold, threshold, erodeElement);
	cv::erode(threshold, threshold, erodeElement);

	cv::dilate(threshold, threshold, dilateElement);
	cv::dilate(threshold, threshold, dilateElement);

}

// track the filtered object
void TrackerPoint::trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed){

	cv::Mat temp;
	threshold.copyTo(temp);

	//these two vectors needed for output of findContours
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;

	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();

		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				cv::Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
				}
				else objectFound = false;


			}
			//let user know you found an object
			if (objectFound == true){
				putText(cameraFeed, "Tracking Object", cv::Point(0, 50), 2, 1, cv::Scalar(0, 255, 0), 2);

				//draw object location on screen
				drawObject(x, y, cameraFeed);
			}

		}
		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", cv::Point(0, 50), 1, 2, cv::Scalar(0, 0, 255), 2);
	}
}


// start to track the point
void TrackerPoint::startTracking(){

	//some boolean variables for different functionality within this
	//program
	bool trackObjects = true;
	bool useMorphologicalFilters = true;

	//Matrix to store each frame of the webcam feed
	cv::Mat cameraFeed;

	//matrix storage for HSV image
	cv::Mat HSV;

	//matrix storage for binary threshold image
	cv::Mat threshold;

	//x and y values for the location of the object
	int x = 0, y = 0;

	//create slider bars for HSV filtering
	createTrackBars();

	//video capture object to acquire webcam feed
	cv::VideoCapture capture;

	//open capture object at location zero (default location for webcam)
	if (!capture.isOpened()){
		capture.open(cameraID);
	}
	//set height and width of capture frame
	capture.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	while (1){

		//store image to matrix
		capture.read(cameraFeed);

		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed, HSV, cv::COLOR_BGR2HSV);

		//filter HSV image between values and store filtered image to
		//threshold matrix
		inRange(HSV, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), threshold);

		//perform morphological operations on thresholded image to eliminate noise
		//and emphasize the filtered object(s)
		if (useMorphologicalFilters)
			applyMorphologicalFilters(threshold);

		//pass in thresholded frame to our object tracking function
		//this function will return the x and y coordinates of the
		//filtered object
		if (trackObjects)
			trackFilteredObject(x, y, threshold, cameraFeed);

		// send signal with information

		//show frames 
		cv::imshow(windowName2, threshold);
		cv::imshow(windowName, cameraFeed);
		cv::imshow(windowName1, HSV);

		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		cv::waitKey(30);
	}

	
}


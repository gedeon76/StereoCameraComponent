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

/// constant for scale factor estimation
const int patternCircleNumber = 44;
const float A_dist = 35;			//  35 mm is the real separation between circles
									//	for the OpenCV asymetric circles pattern with 44 circles

/// These points are used to build the Mi's according to
/// the second method described in Lourakis'13 article 
/// titled "Accurate scale Factor Estimation in 3D Reconstruction"
const cv::Point3f M1(0,0,0);
const cv::Point3f M2(A_dist, 0, 0);
const cv::Point3f M3(0.5*A_dist, 0.5*A_dist, 0);
const cv::Point3f M4(0, A_dist, 0);
const cv::Point3f M5(2*A_dist, 0, 0);
const cv::Point3f M6(1.5*A_dist, 0.5*A_dist, 0);
const cv::Point3f M7(A_dist, A_dist, 0);
const cv::Point3f M8(0.5*A_dist, 1.5*A_dist, 0);
const cv::Point3f M9(0, 2*A_dist, 0);
const cv::Point3f M10(3*A_dist, 0, 0);
const cv::Point3f M11(2.5*A_dist, 0.5*A_dist, 0);
const cv::Point3f M12(2*A_dist, A_dist, 0);
const cv::Point3f M13(1.5*A_dist, 1.5*A_dist, 0);
const cv::Point3f M14(A_dist, 2*A_dist, 0);
const cv::Point3f M15(0.5*A_dist, 2.5*A_dist, 0);
const cv::Point3f M16(0, 3*A_dist, 0);
const cv::Point3f M17(3.5*A_dist, 0.5*A_dist, 0);
const cv::Point3f M18(3*A_dist, A_dist, 0);
const cv::Point3f M19(2.5*A_dist, 1.5*A_dist, 0);
const cv::Point3f M20(2*A_dist, 2*A_dist, 0);
const cv::Point3f M21(1.5*A_dist, 2.5*A_dist, 0);
const cv::Point3f M22(A_dist, 3*A_dist, 0);
const cv::Point3f M23(0.5*A_dist, 3.5*A_dist, 0);
const cv::Point3f M24(0, 4*A_dist, 0);
const cv::Point3f M25(3.5*A_dist, 1.5*A_dist, 0);
const cv::Point3f M26(3*A_dist, 2*A_dist, 0);
const cv::Point3f M27(2.5*A_dist, 2.5*A_dist, 0);
const cv::Point3f M28(2*A_dist, 3*A_dist, 0);
const cv::Point3f M29(1.5*A_dist, 3.5*A_dist, 0);
const cv::Point3f M30(A_dist, 4*A_dist, 0);
const cv::Point3f M31(0.5*A_dist, 4.5*A_dist, 0);
const cv::Point3f M32(0, 5*A_dist, 0);
const cv::Point3f M33(3.5*A_dist, 2.5*A_dist, 0);
const cv::Point3f M34(3*A_dist, 3*A_dist, 0);
const cv::Point3f M35(2.5*A_dist, 3.5*A_dist, 0);
const cv::Point3f M36(2*A_dist, 4*A_dist, 0);
const cv::Point3f M37(1.5*A_dist, 4.5*A_dist, 0);
const cv::Point3f M38(A_dist, 5*A_dist, 0);
const cv::Point3f M39(3.5*A_dist, 3.5*A_dist, 0);
const cv::Point3f M40(3*A_dist, 4*A_dist, 0);
const cv::Point3f M41(2.5*A_dist, 4.5*A_dist, 0);
const cv::Point3f M42(2*A_dist, 5*A_dist, 0);
const cv::Point3f M43(3.5*A_dist, 4.5*A_dist, 0);
const cv::Point3f M44(3*A_dist, 5*A_dist, 0);


// asymetric circles distribution
const double Mi_Positions[] = { M1.x, M1.y, M1.z,
	M2.x, M2.y, M2.z, M3.x, M3.y, M3.z, M4.x, M4.y, M4.z,
	M5.x, M5.y, M5.z, M6.x, M6.y, M6.z, M7.x, M7.y, M7.z, M8.x, M8.y, M8.z, M9.x, M9.y, M9.z, 
	M10.x, M10.y, M10.z, M11.x, M11.y, M11.z, M12.x, M12.y, M12.z, M13.x, M13.y, M13.z, M14.x, M14.y, M14.z, M15.x, M15.y, M15.z, M16.x, M16.y, M16.z,
	M17.x, M17.y, M17.z, M18.x, M18.y, M18.z, M19.x, M19.y, M19.z, M20.x, M20.y, M20.z, M21.x, M21.y, M21.z, M22.x, M22.y, M22.z, M23.x, M23.y, M23.z, M24.x, M24.y, M24.z,
	M25.x, M25.y, M25.z, M26.x, M26.y, M26.z, M27.x, M27.y, M27.z, M28.x, M28.y, M28.z, M29.x, M29.y, M29.z, M30.x, M30.y, M30.z, M31.x, M31.y, M31.z, M32.x, M32.y, M32.z,
	M33.x, M33.y, M33.z, M34.x, M34.y, M34.z, M35.x, M35.y, M35.z, M36.x, M36.y, M36.z, M37.x, M37.y, M37.z, M38.x, M38.y, M38.z,
	M39.x, M39.y, M39.z, M40.x, M40.y, M40.z, M41.x, M41.y, M41.z, M42.x, M42.y, M42.z,
	M43.x, M43.y, M43.z, M44.x, M44.y, M44.z};


///// this structure save the points found for the asymetric pattern
//struct circlePatternInfo
//{
//	int circleID;
//	cv::Point2f circlePosition;
//	cv::Point3f circle3DPosition;
//};
//
///// This structure save all the circles found per each calibration image
//struct circlesDataPerImage
//{
//	int cameraID;
//	int imageID;
//	std::vector<circlePatternInfo> circlesData;
//};

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
	double pixelpermmX;
	double pixelpermmY;
	double fov_X;
	double fov_Y;
	double focalLength;
	double principalPointX;
	double principalPointY;
	double aspectRatio;
	double stereoBaseline;
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
/// Master Thesis Titled: Range determination for Mobile robot using an omnidirectional camera
struct E_MatricesCandidates{

	double qE;
	cv::Mat E_matrix;

};

typedef cameraUsefulData cameraData;
typedef std::vector<cameraData> cameraParameters;
typedef sortMatch sortedMatches;
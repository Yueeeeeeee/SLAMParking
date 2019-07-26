#include <opencv2/opencv.hpp>
#include<iterator>



using namespace std;
using namespace cv;


namespace pth{

//The next two constants are valid for the Microsoft LiveCam HD 3000 in 424 * 240 pixel resolution mode.
//Theoretically this can be slightly different for each camera, but practically it should not mather for our application
//those values where aquired with the Opencv standard example program for camera calibration
//https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
Mat cameraMatrix =
		(Mat1d(3, 3) << 2.8419682583562758e+02, 0.0, 212.0, 0.0, 2.8419682583562758e+02, 120.0, 0.0, 0.0, 1.0);
Mat distCoeffs =
		(Mat1d(1, 5) << 2.2112127459840783e-01, -9.7795764741287716e-01, 0.0, 0.0, 9.6134925621043477e-01);

//this constant was aquired with a very small standalone programm and a custom checkerboard.
//The numbers in the Homography-Matrix are only valid as long as the camera-mointing-position on the rover is not changed.
Mat inverseHomography =
		(Mat1d(3, 3) << 17.538539489614877, -0.82001653537480601, 0.33615747624611997, -0.45541653173535068, -14.581050368587377, 16.248381063513335, -0.021208579719434899, 0.65021160721307936, 0.27680260466006223);


/*
 *The Homography was aquired in a way so that the coordinate system
 *after the transform looks as follows:
 *
 *				^y
 *				|
 * 				|
 * 				|
 * 				|
 * 				|
 * 				|
 *				|
 *--------------o-------------->x
 * 			  -----
 * 			 0     0
 * 			 |rover|  ^
 * 			 |     |  |
 * 			 0     0
 * 			  -----
 *
 *
 * */

cv::Point2d projectOntoPlane(int x, int y) {

	vector<Point2f> pixelPoint;
	pixelPoint.push_back(Point2f(1.0 * x, 1.0 * y));

	vector<Point2f> undistPoint;
	undistortPoints(pixelPoint, undistPoint, cameraMatrix, distCoeffs);

	Vec3d tempPoint(undistPoint[0].x, undistPoint[0].y, 1.0);
	Mat_<double> imagePoint(tempPoint);

	Mat backProjected = inverseHomography * imagePoint;

	return cv::Point2d(
			backProjected.at<double>(0, 0) / backProjected.at<double>(0, 2),
			backProjected.at<double>(0, 1) / backProjected.at<double>(0, 2));

}

vector<Point2d> projectOntoPlane(vector<Point2i> &spline){
	vector<Point2d>result;
	for(Point2i &p:spline){
		result.push_back(projectOntoPlane(p.x,p.y));
	}
	return result;
}

}

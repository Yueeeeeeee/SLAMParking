/*
 * LaneDetection.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: martin, christian
 */
#include "LaneDetection.h"
#include <opencv2/opencv.hpp>
#include <iterator>

//no class, no compilation to object-format.
//only include the source directly
#include "LaneDetectionAccessories/DebugWriter.cpp"
#include "LaneDetectionAccessories/ContourSplineHelper.cpp" //acces via namespace csh

using namespace std;
using namespace cv;

// Calculation parameters

// Image size
const int COLS = 640; // Number of pixels of the image
const int ROWS = 360; // Number of pixels of the image
const float COLS_ROI_PORTION = 0.7;
const float ROWS_ROI_PORTION = 0.15;
const int COLS_ROI = COLS_ROI_PORTION * COLS; // Number of pixels of the ROI
const int ROWS_ROI = ROWS_ROI_PORTION * ROWS; // Number of pixels of the ROI
const Rect ROI = Rect(COLS * 0.15, ROWS * 0.78, COLS_ROI, ROWS_ROI); // Rect of the ROI

const int MIN_CONTOUR_LENGTH = 40; // Min length to be a valid contour
const int MIN_CONTOUR_Y_SPANN = 20; // Min height to be a valid contour
const int SPLINE_DISTANCE = 20; // Approx. distance between points in a spline

//KERNEL_SIZE and ADAPTIVE_TRESHOLD_WINDOW_SIZE have also a drastical influence on the runtime
const int KERNEL_SIZE = 5;	// Kernel size for the image blurring
const int ADAPTIVE_TRESHOLD_WINDOW_SIZE = 31; // Window size for the lane color filtering
const int ADAPTIVE_TRESHOLD_SUBSTRACT = -20; // Treshold difference for the lane color filtering

Mat* PERSPECTIVE = nullptr;	// Transformation matrix for perspective transform

Scalar COLORS[] = { Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255),
		Scalar(255, 255, 0), Scalar(0, 255, 255), Scalar(255, 255, 0) }; // Some colors for visualization

LaneDetection::LaneDetection(bool createResultFrame, int width, int height,
		PrecedingRover_ts &roverContour) {
	this->roverContour = &roverContour;

	this->imageWidth = width;
	this->imageHeight = height;
	this->createResultFrame = createResultFrame;

	std::cout << "LaneDetection Object constructed" << std::endl;
}

LaneDetection::~LaneDetection() {
}

void LaneDetection::processImage(Mat currentFrame) {
	static Mat currentFrameWithoutRover;
	static Mat cropedResized;
	static Mat filteredLaneColor;
	static Mat erasedRover;
	static Mat smoothedEdges;
	static Mat detectedEdges;
	static Mat selectedArea;
	static Mat lanesDrawn;
	static Mat temp;
	vector<float> distances;
	vector<vector<Point2i>> contours, contours2, splines;

	//invalidate all results from the last run (code is assumed to run sequentially)
	this->resultData.curve = false;
	this->resultData.detectedLeft = false;
	this->resultData.detectedRight = false;
	this->resultData.curveRadius = 0.0;
	this->resultData.distanceLeft = numeric_limits<float>::quiet_NaN();
	this->resultData.distanceRight = numeric_limits<float>::quiet_NaN();
	this->resultData.yawAngle = 0.0;
	// Used for raw video output to a file
	this->baseFrame = currentFrame;
	//the more resize, the faster the computation
	//crop to only the lower part of the image where the street is supposed to be
#ifdef MEASSURE_EXECUTION_TIME
	auto startCropAndResizeImage = std::chrono::high_resolution_clock::now();
#endif
	cropAndResizeImage(currentFrame, cropedResized);
#ifdef MEASSURE_EXECUTION_TIME
	auto finishCropAndResizeImage = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedCropAndResizeImage = finishCropAndResizeImage - startCropAndResizeImage;
	std::cout << "- Crop and Resize:\t" << elapsedCropAndResizeImage.count() * 1000 << " ms" << std::endl;
#endif

	// based on the color the lane is filtered
	// Input is a RGB image, output a binary image
#ifdef MEASSURE_EXECUTION_TIME
	auto startFilterLaneColor = std::chrono::high_resolution_clock::now();
#endif
	filterLaneColor(cropedResized, filteredLaneColor);
#ifdef MEASSURE_EXECUTION_TIME
	auto finishFilterLaneColor = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedFilterLaneColor = finishFilterLaneColor - startFilterLaneColor;
	std::cout << "- Filter Lanes:\t\t" << elapsedFilterLaneColor.count() * 1000 << " ms" << std::endl;
#endif

	// Using the coordinates to erase the rover from the image
#ifdef MEASSURE_EXECUTION_TIME
	auto startErasePrecedingRover = std::chrono::high_resolution_clock::now();
#endif
	ErasePrecedingRover(filteredLaneColor, erasedRover);
#ifdef MEASSURE_EXECUTION_TIME
	auto finishErasePrecedingRover = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedErasePrecedingRover = finishErasePrecedingRover - startErasePrecedingRover;
	std::cout << "- Erase Preceding Rover:" << elapsedErasePrecedingRover.count() * 1000 << " ms" << std::endl;
#endif

#ifdef MEASSURE_EXECUTION_TIME
	auto startgetContours = std::chrono::high_resolution_clock::now();
#endif
	getContours(erasedRover, contours2);
#ifdef MEASSURE_EXECUTION_TIME
	auto finishgetContours = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedgetContours = finishgetContours - startgetContours;
	std::cout << "- Get Lane Contours:\t" << elapsedgetContours.count() * 1000 << " ms" << std::endl;
#endif

#ifdef MEASSURE_EXECUTION_TIME
	auto startTransform = std::chrono::high_resolution_clock::now();
#endif
	if (createResultFrame) {
		transformImage(erasedRover, smoothedEdges);
		cvtColor(smoothedEdges, lanesDrawn, COLOR_GRAY2BGR);
	}

	transformPoints(contours2, contours);
#ifdef MEASSURE_EXECUTION_TIME
	auto finishTransform = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedTransform = finishTransform - startTransform;
	std::cout << "- Transform:" << elapsedTransform.count() * 1000 << " ms" << std::endl;
#endif

#ifdef MEASSURE_EXECUTION_TIME
	auto startGetSplines = std::chrono::high_resolution_clock::now();
#endif
	csh::getSplines(contours, splines);
#ifdef MEASSURE_EXECUTION_TIME
	auto finishGetSplines = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedGetSplines = finishGetSplines - startGetSplines;
	std::cout << "- Get Splines:\t\t" << elapsedGetSplines.count() * 1000 << " ms" << std::endl;
#endif

#ifdef MEASSURE_EXECUTION_TIME
	auto startGetDistance = std::chrono::high_resolution_clock::now();
#endif
	getLaneDistance(splines, distances, lanesDrawn);
#ifdef MEASSURE_EXECUTION_TIME
	auto finishGetDistance = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsedGetDistance = finishGetDistance - startGetDistance;
	std::cout << "- Get Distance:\t\t" << elapsedGetDistance.count() * 1000 << " ms" << std::endl;
#endif

	if (false /*distances[0] != 0.0*/) {// Only use right line, more stable but less reliable
		this->resultData.detectedLeft = true;
		this->resultData.distanceLeft = distances[0];
	}

	if (distances[1] != 0.0) {
		this->resultData.detectedRight = true;
		this->resultData.distanceRight = distances[1];
	}

	this->resultData.curve = distances[2] != 0.0;
	this->resultData.curveRadius = distances[2];
	this->resultData.yawAngle = distances[3];

	if (createResultFrame) {

		for (uint i = 0; i < contours.size(); i++) {
			for (uint j = 0; j < contours[i].size() - 1; j++) {
				line(lanesDrawn, contours[i][j], contours[i][j + 1],
						COLORS[i % 6], 2);
			}
		}

		// Stick multiple Mats into one
		int cols = cropedResized.cols + lanesDrawn.cols;
		int rows = max(
				cropedResized.rows + filteredLaneColor.rows + erasedRover.rows,
				lanesDrawn.rows);
		Mat3b toShow(rows, cols, Vec3b(0, 0, 0));
		cropedResized.copyTo(
				toShow(Rect(0, 0, cropedResized.cols, cropedResized.rows)));
		Mat tmp;
		cvtColor(erasedRover, tmp, COLOR_GRAY2BGR);
		tmp.copyTo(toShow(Rect(0, cropedResized.rows, tmp.cols, tmp.rows)));
		lanesDrawn.copyTo(
				toShow(
						Rect(cropedResized.cols, 0, lanesDrawn.cols,
								lanesDrawn.rows)));

		this->resultFrame = toShow;
	}
}

Mat LaneDetection::getResultFrame() {
	return this->resultFrame;
}

Mat LaneDetection::getBaseFrame() {
	return this->baseFrame;
}

LDValues LaneDetection::getResultData() {
	return this->resultData;
}

/**
 * Filters the image to only contain the lane color
 */
void LaneDetection::filterLaneColor(Mat &image, Mat &result) {
	Mat tmp, tmp2;
	cvtColor(image, tmp, COLOR_RGB2HLS);
	extractChannel(tmp, tmp2, 1);
	cv::adaptiveThreshold(tmp2, result, 255, ADAPTIVE_THRESH_MEAN_C,
			THRESH_BINARY, ADAPTIVE_TRESHOLD_WINDOW_SIZE,
			ADAPTIVE_TRESHOLD_SUBSTRACT);
}

/**
 * Erases the preceding rover, should it be inside the ROI
 */
void LaneDetection::ErasePrecedingRover(Mat &image, Mat &result) {
	result = image.clone();

	int numberOfPoints = 4; // hard coded, best coded
	Point currentContour[numberOfPoints];
	memcpy(currentContour, roverContour->getContour(), sizeof(Point) * 4);

	if (!(roverContour->isRoverRecognized())) {
		return;
	}
	cout << "Rover detected\n";
	for (int i = 0; i < numberOfPoints; i++) {
		currentContour[i].x = min(max(currentContour[i].x - ROI.x, 0),
				ROI.width);
		currentContour[i].y = min(max(currentContour[i].y - ROI.y, 0),
				ROI.height);
	}

	fillConvexPoly(result, currentContour, numberOfPoints, Scalar(0), LINE_8);
}

/**
 * Crops and resizes the image
 */
void LaneDetection::cropAndResizeImage(Mat &image, Mat &result) {
	//flip(image, image, 0);
	resize(image, result, Size(COLS, ROWS));
	result(ROI).copyTo(result);
	this->resizedImageHeight = result.rows;
	this->resizedImageWidth = result.cols;
}

/**
 * Gets the contours in the image ignoring invalid ones
 */
void LaneDetection::getContours(Mat &image, vector<vector<Point2i>> &contours) {
	GaussianBlur(image, image, Size(KERNEL_SIZE, KERNEL_SIZE), KERNEL_SIZE);

	findContours(image, contours, 1, 1);
	contours.erase(
			remove_if(contours.begin(), contours.end(),
					[](vector<Point2i> vec) {return (vec.size() < MIN_CONTOUR_LENGTH)||(abs(vec[csh::getLowestPoint(vec)].y-vec[csh::getHighestPoint(vec)].y)<MIN_CONTOUR_Y_SPANN);}),
			contours.end());
}

/**
 * Creates a string containing the results
 */
string resultString(LDValues &result) {
	ostringstream out;
	out.precision(3);
	out << fixed;
	out << "lDist: ";
	result.detectedLeft ? out << result.distanceLeft : out << "_____";
	out << " \trDist: ";
	result.detectedRight ? out << result.distanceRight : out << "_____";
	out << " \tangle: ";
	if (!(result.yawAngle < 0.0)) {
		out << "+";
	}
	out << result.yawAngle;
	out << " \tcurve: ";
	result.curve ? out << result.curveRadius : out << "_____";
	return out.str();
}

/**
 * Creates the transformation matrix used to to do a perspective transformation to the birds-eye view
 */
void LaneDetection::createTransformationMatrix() {
	Point2f source[4];
	source[0] = Point2f(COLS_ROI * 0, ROWS_ROI * 0);
	source[1] = Point2f(COLS_ROI * 1, ROWS_ROI * 0);
	source[2] = Point2f(COLS_ROI * 1, ROWS_ROI * 1);
	source[3] = Point2f(COLS_ROI * 0, ROWS_ROI * 1);

	Point2f dst[4];
	dst[0] = Point2f(COLS_ROI * 0, ROWS_ROI * 0);
	dst[1] = Point2f(COLS_ROI * 1, ROWS_ROI * 0);
	dst[2] = Point2f(COLS_ROI * 0.7, ROWS_ROI * 2.5);
	dst[3] = Point2f(COLS_ROI * 0.3, ROWS_ROI * 2.5);

	PERSPECTIVE = new Mat();
	getPerspectiveTransform(source, dst).copyTo(*PERSPECTIVE);
}

/**
 * Transforms the input image to a birds-eye view
 */
void LaneDetection::transformImage(Mat &image, Mat &result) {

	if (PERSPECTIVE == nullptr) {
		createTransformationMatrix();
	}

	warpPerspective(image, result, *PERSPECTIVE,
			Size(COLS_ROI * 1, ROWS_ROI * 3));
}

/**
 * Transforms the given Points to a birds-eye view
 */
void LaneDetection::transformPoints(vector<vector<Point2i>> &contoursIn,
		vector<vector<Point2i>> &contoursOut) {

	if (PERSPECTIVE == nullptr) {
		createTransformationMatrix();
	}

	contoursOut.clear();
	for (vector<Point2i> vec : contoursIn) {
		vector<Point2f> tmp;

		for (Point2i p : vec) {
			tmp.push_back((Point2f) p);
		}

		perspectiveTransform(tmp, tmp, *PERSPECTIVE);

		vec.clear();
		for (Point2f p : tmp) {
			vec.push_back((Point2i) p);
		}
		contoursOut.push_back(vec);
	}
}

/**
 * Returns the distance between two points
 */
float distance(Point p1, Point p2) {
	return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

/**
 * Calculates the angle of a line
 */
float slope(Vec4i line) {
	if (line[1] < line[3]) {
		return atan2(line[3] - line[1], line[2] - line[0]);
	} else {
		return atan2(line[1] - line[3], line[0] - line[2]);
	}
}

float slope(Point p1, Point p2) {
	if (p1.y < p2.y) {
		return atan2(p2.y - p1.y, p2.x - p1.x);
	} else {
		return atan2(p1.y - p2.y, p1.x - p2.x);
	}
}

/**
 * Projects the spline and returns the distance to the target
 */
tuple<float, float, float> LaneDetection::projectSpline(vector<Point> &spline,
		Point target, Mat &image) {
	vector<float> angle;
	float avgDerivative = 0.0;
	int totalDerivatives = 0;
	for (uint i = 0; i < spline.size() - 1; i++) {
		float newAngle = slope(
				Vec4i(spline[i + 1].x, spline[i + 1].y, spline[i].x,
						spline[i].y));
		angle.push_back(newAngle);
	}
	for (uint i = 0; i < spline.size() - 2; i++) {
		float derivative = (angle[i + 1] - angle[i]);

		float dist = distance(spline[i], spline[i + 1]);
		if (abs(derivative) > 0.5 || dist < 5) {
			// Discard invalid values
			if (createResultFrame) {
				putText(image, "---", spline[i + 1], FONT_HERSHEY_COMPLEX_SMALL,
						0.5, Scalar(0, 0, 255), 1);
			}
		} else {
			if (createResultFrame) {
				ostringstream ss;
				ss << derivative;
				putText(image, ss.str(), spline[i + 1],
						FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(255, 255, 255),
						1);
			}
			avgDerivative += derivative;
			totalDerivatives++;
		}
	}
	avgDerivative /= totalDerivatives;

	if (createResultFrame) {
		circle(image, target, 6, Scalar(0, 0, 255), 5);
	}

	Point newPoint;
	Point currentPoint = (spline[0] + spline[1] + spline[2]) / 3;
	float currentDirection = (angle[0] + angle[1] + angle[2]) / 3;
	float minDistance = distance(currentPoint, target);
	int i = 0;
	while (i < 20) {	// Prevent infinite/slowly converging scenarios
		i++;
		newPoint = Point(
				currentPoint.x + cos(currentDirection) * SPLINE_DISTANCE,
				currentPoint.y + sin(currentDirection) * SPLINE_DISTANCE);

		if (createResultFrame) {
			line(image, currentPoint, newPoint, Scalar(127, 255, 255), 4);
		}

		float newDistance = distance(newPoint, target);

		if (newDistance > minDistance) {
			// the new point is further away than the previous point -> we found the closest approach
			if (currentPoint.x < target.x) {
				minDistance = -minDistance;
			}
			minDistance *= (448.0 / COLS_ROI); // PID-Controller is adjusted for a 448x162 ROI

			if (createResultFrame) {
				ostringstream ss;
				ss << minDistance << "m";
				putText(image, ss.str(),
						((currentPoint + target) / 2) - Point(0, 20),
						FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(0, 0, 255), 1);
				circle(image, currentPoint, 6, Scalar(0, 255, 255), 3);
				circle(image, newPoint, 6, Scalar(0, 255, 0), 3);
				line(image, currentPoint, target, Scalar(0, 0, 255), 4);
			}
			goto end;
		} else {
			if (createResultFrame) {
				circle(image, newPoint, 6, Scalar(0, 255, 0), 3);
			}
		}

		currentDirection -= avgDerivative;
		currentPoint = newPoint;
		minDistance = newDistance;
	}
	end: return make_tuple(minDistance, avgDerivative, currentDirection); // distance, curvature, yaw angle
}

/**
 * Finds the distance of a spline to the vehicle
 */
tuple<float, float, float> LaneDetection::getDistance(vector<Point> &spline,
		Mat &image) {
	return projectSpline(spline, Point(COLS_ROI / 2, ROWS_ROI * 3.0), image);
}

/*
 * Finds the distance to the closest spline on each side
 */
void LaneDetection::getLaneDistance(vector<vector<Point>> &splines,
		vector<float> &distances, Mat &image) {
	distances.clear();
	// Find closest approach on either side
	tuple<float, float, float> minDistLeft = make_tuple(0.0, 0.0, 0.0);
	tuple<float, float, float> minDistRight = make_tuple(0.0, 0.0, 0.0);
	for (vector<Point> spline : splines) {
		tuple<float, float, float> dist = getDistance(spline, image);
		if (get<0>(dist) < 0
				&& (get<0>(dist) > get<0>(minDistLeft)
						|| get<0>(minDistLeft) == 0.0)) {
			minDistLeft = dist;
		} else if (get<0>(dist) > 0
				&& (get<0>(dist) < get<0>(minDistRight)
						|| get<0>(minDistRight) == 0.0)) {
			minDistRight = dist;
		}
	}
	distances.push_back(-get<0>(minDistLeft));
	distances.push_back(get<0>(minDistRight));
	distances.push_back(
			(get<1>(minDistLeft) + get<1>(minDistRight))
					/ ((get<1>(minDistLeft) != 0.0
							&& get<1>(minDistRight) != 0.0) ? 2 : 1));
	distances.push_back(
			(get<2>(minDistLeft) + get<2>(minDistRight))
					/ ((get<2>(minDistLeft) != 0.0
							&& get<2>(minDistRight) != 0.0) ? 2 : 1));
}

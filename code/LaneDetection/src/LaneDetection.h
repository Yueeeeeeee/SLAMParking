/*
 * LaneDetection.h
 *
 *  Created on: Jan 14, 2018
 *      Author: martin
 */

#ifndef SRC_LANEDETECTION_H_
#define SRC_LANEDETECTION_H_

#include <opencv2/opencv.hpp>
#include "VideoProviders/VideoProvider.h"
#include "VideoConsumers/VideoConsumer.h"
#include "ClientServer/Data.h"
#include "PrecedingRover_ts.h"

using namespace std;
using namespace cv;

/**
 * Class that handles the lane detection
 */
class LaneDetection {
public:

	LaneDetection(bool createResultFrame, int witdh, int height);
	LaneDetection(bool createResultFrame, int witdh, int height,
			PrecedingRover_ts &roverContour);
	virtual ~LaneDetection();
	void processImage(Mat image);
	Mat getResultFrame();
	Mat getBaseFrame();
	LDValues getResultData();

private:
	bool createResultFrame = false;

	PrecedingRover_ts* roverContour;

	int imageWidth;
	int imageHeight;
	int resizedImageWidth = 0;
	int resizedImageHeight = 0;

	Mat resultFrame;
	Mat baseFrame;
	LDValues resultData;

	void transformToUncropedUnresized(vector<Point2i>&spline);
	void transformToCropedResized(vector<Point2i>&spline);
	void filterLaneColor(Mat &image, Mat &result);
	void cropAndResizeImage(Mat &image, Mat &result);
	void ErasePrecedingRover(Mat &image, Mat &result);
	void getContours(Mat &image,
			vector<vector<Point>> &contours);
	void transformPoints(vector<vector<Point2i>> &contoursIn,
			vector<vector<Point2i>> &contoursOut);
	void transformImage(Mat &image, Mat &result);
	tuple<float, float, float> projectSpline(vector<Point> &spline, Point target,
			Mat &image);
	tuple<float, float, float> getDistance(vector<Point> &spline, Mat &image);
	void getLaneDistance(vector<vector<Point>> &splines,
			vector<float> &distances, Mat &image);
	void createTransformationMatrix();
};

#endif /* SRC_LANEDETECTION_H_ */

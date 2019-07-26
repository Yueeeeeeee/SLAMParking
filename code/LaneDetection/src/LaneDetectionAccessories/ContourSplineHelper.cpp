/*
 * ContourHelper.cpp
 *
 *  Created on: Jun 21, 2018
 *      Author: martin
 */

#include <opencv2/opencv.hpp>
#include <iterator>

//////////////////////////
// Note: Opencv coordinate system: (0,0) is at the upper left corner of the image
////////////////////////

using namespace std;
using namespace cv;

namespace csh {

void getSplines(vector<vector<Point>> &contours,
		vector<vector<Point>> &splines);
void cleanSpline(vector<Point> &spline);
int getLowestPoint(vector<Point2i> &contour);
int getHighestPoint(vector<Point2i> &contour);
float slope(Vec4i line);
float slope(Point p1, Point p2);
double meanX(vector<Point2i> &spline);

const int SPLINE_DISTANCE = 10;

/**
 * Finds a spline for every contour by following both "sides" from the bottom to the top
 */
void getSplines(vector<vector<Point>> &contours,
		vector<vector<Point>> &splines) {
	splines.clear();
	for (vector<Point> vec : contours) {
		int lowest = getHighestPoint(vec); //lowest point in the image has highest coordinate
		int side1 = lowest;
		int side2 = lowest;
		int currentY = vec[lowest].y;
		int nextStep = SPLINE_DISTANCE / 2;
		Point prevPoint = vec[lowest];
		vector<Point> spline;
		while (true) {
			currentY -= nextStep;
			if (side1 >= (int) vec.size()) {
				side1 = 0;
			}
			do {
				side1++;
			} while (side1 < (int) vec.size() && vec[side1].y > currentY);
			if (side2 < 0) {
				side2 = vec.size() - 1;
			}
			do {
				side2--;
			} while (side2 >= 0 && vec[side2].y > currentY);
			if (side1 >= (int) vec.size() || side2 < 0) {
				goto end;
			}
			int centerX = (vec[side1].x + vec[side2].x) / 2;
			int centerY = (vec[side1].y + vec[side2].y) / 2;
			Point newPoint = Point(centerX, centerY);
			spline.push_back(newPoint);

			nextStep = SPLINE_DISTANCE
					* sin(
							slope(
									Vec4i(prevPoint.x, prevPoint.y, newPoint.x,
											newPoint.y)));
			prevPoint = newPoint;
		}
		end: Point highest = vec[getLowestPoint(vec)];
		spline.push_back(highest);

		cleanSpline(spline);
		if (spline.size() > 4) {
			splines.push_back(spline);
		}
	}
}

/**
 * Not particularly useful, but might break everything if removed
 */
void cleanSpline(vector<Point> &spline) {
	if (spline.size() < 3) {
		return;
	}
	vector<float> angle;
	vector<float> derivative;
	float avgDerivative = 0.0;

	for (uint i = 0; i < spline.size() - 1; i++) {
		angle.push_back(
				slope(
						Vec4i(spline[i + 1].x, spline[i + 1].y, spline[i].x,
								spline[i].y)));
	}
	for (uint i = 0; i < spline.size() - 2; i++) {
		derivative.push_back(angle[i + 1] - angle[i]);
		avgDerivative += derivative[i];
	}

	if (abs(derivative[0])
			> (abs(derivative[1]) + abs(derivative[2])) / 2 - 20) {
		spline.erase(spline.begin());
		angle.erase(angle.begin());
		avgDerivative -= derivative[0];
		derivative.erase(derivative.begin());
	}

}

int getLowestPoint(vector<Point2i> &contour) {
	int index = 0;
	int lowest = 0;
	for (uint i = 0; i < contour.size(); i++) {
		if (contour[i].y < lowest) {
			lowest = contour[i].y;
			index = i;
		}
	}
	return index;
}

int getHighestPoint(vector<Point2i> &contour) {
	int index = 0;
	int highest = 0;
	for (uint i = 0; i < contour.size(); i++) {
		if (contour[i].y > highest) {
			highest = contour[i].y;
			index = i;
		}
	}
	return index;
}

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

double meanX(vector<Point2i> &spline) {
	int sum = 0;
	for (Point2i &p : spline) {
		sum += p.x;
	}
	return sum / spline.size();
}

}

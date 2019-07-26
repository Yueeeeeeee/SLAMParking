//
// Created by fabian on 16.06.18.
//

#include "PrecedingRoverDetection.h"
#include <iostream>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

using namespace cv;
using namespace std;

//Parameter for QR Code
const double SIZE_OF_QR_CODE = 100; //height=width in mm

PrecedingRoverDetection::PrecedingRoverDetection(Frame_ts &currentFrame, PrecedingRover_ts *roverContour) {
	this->frame = &currentFrame;
	this->precedingRover = roverContour;
}

void PrecedingRoverDetection::Run() {
	int64 start = cv::getTickCount();
	int counter = 0;

	while (true) {
		cv::Mat currentFrame = this->frame->getFrame().clone();
		if (currentFrame.empty())
			continue;
		getContourToErase(currentFrame);

		counter++;
		if (counter % 30 == 0) {
			counter = 0;
			double timeDiff = (cv::getTickCount() - start) / cv::getTickFrequency();
			start = cv::getTickCount();
			std::cout << "\t\t\t\t\t\tQR-Code running at: " << 30 / timeDiff << " fps\n";
		}
	}
}

bool pointYComparer(Point i, Point j) {
	return (i.y < j.y);
}

/**
 * returns the convex hull of the preceding rover. Result should only be used for debugging issues.
 * If precedingRoverRecognized is false, precedingRoverId and precedingRoverDistance might be trash.
 */
void PrecedingRoverDetection::getContourToErase(Mat &im) {

	// 1. Detect QR code an extract coordinates
	// https://www.learnopencv.com/barcode-and-qr-code-scanner-using-zbar-and-opencv/

	typedef struct {
		string type;
		string data;
		vector<Point> location;
	} decodedObject;

	vector<decodedObject> decodedObjects;

	// Create zbar scanner
	zbar::ImageScanner scanner;

	// Configure scanner
	scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

	// Convert image to grayscale
	Mat imGray;
	cvtColor(im, imGray, CV_BGR2GRAY);

	// Wrap image data in a zbar image
	zbar::Image image(im.cols, im.rows, "Y800", (uchar *) imGray.data, im.cols * im.rows);

	// Scan the image for QRCodes
	if(scanner.scan(image) < 0) {
		cout << "QR-Scanner error\n";
	}

	// Print results
	for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
		decodedObject obj;

		obj.type = symbol->get_type_name();
		obj.data = symbol->get_data();

		// Obtain location
		for (int i = 0; i < symbol->get_location_size(); i++) {
			obj.location.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
		}

		decodedObjects.push_back(obj);
	}

	if (decodedObjects.size() == 0) {
		// No QR Code found
		framesWithoutRover++;
		if (framesWithoutRover >= 3)
			this->precedingRover->setRoverRecognized(false);

		return;
	}
	framesWithoutRover = 0;

	// Loop over all decoded objects. Usually there should be only one object (= rover)
	for (unsigned int i = 0; i < decodedObjects.size(); i++) {
		vector<Point> points = decodedObjects[i].location;
		vector<Point> hull;
		// If the points do not form a quad, find convex hull
		if (points.size() > 4) {
			convexHull(points, hull);
		}
		else if(points.size()==4){
			hull = points;
		}
		else{//smaller than 4. We dont know how to proceed
			return;
		}

		Moments M = moments(hull);

		// Number of points in the convex hull
		int n = hull.size();

		//for(int j = 0; j < n; j++)
		//{
		//    line(result, hull[j], hull[ (j+1) % n], Scalar(0,255,0), 3);
		//}

		Point center = Point(M.m10 / M.m00, M.m01 / M.m00);

		//circle(result, center, 3, Scalar(0,255,0), 3);

		Point* currentContour = new Point[4]();
		for (int j = 0; j < n && j < 4; j++) {
			float factorX = 1.5;
			float factorY = 1.5;

			Point pt1 = Point((hull[j].x - center.x) * factorX + center.x, (hull[j].y - center.y) * factorY + center.y);
			currentContour[j] = pt1;
		}
		this->precedingRover->setContour(currentContour);
	}

	// 2. Get Coordinates in world coordinate system

	// calc average QR Code height --> ((two higher y-values) - (two lower y-values))/2
	vector<Point> locationVector(decodedObjects[0].location);
	std::sort(locationVector.begin(), locationVector.end(), pointYComparer);
	double qrCodeHeight = ((locationVector[0].y + locationVector[1].y) - (locationVector[2].y + locationVector[3].y)) / (double) 2;
	// formular: https://stackoverflow.com/questions/45364856/how-to-get-the-distance-of-the-object-and-how-to-use-camera-calibration-matrix-c
	//(brennweite[mm]*höhe[mm]*auflösungy[px]) / (höhe[px]*sensorheight[px]
	double c = 0.87 / 35; // Focal-length / Sensor-Height, focal length is from datasheet, sensor height has been guessed
	int imageHeight = frame->getFrame().size[1];
	int precedingRoverDistance = round(abs(c * (SIZE_OF_QR_CODE * imageHeight) / (qrCodeHeight)));

	int precedingRoverId = -1;
	try {
		precedingRoverId = std::stoi(decodedObjects[0].data);
	} catch (const std::exception& e) {
		//swallow exception
	}
	this->precedingRover->setDistance(precedingRoverDistance);
	this->precedingRover->setId(precedingRoverId);
	this->precedingRover->setRoverRecognized(true);

	return;
}


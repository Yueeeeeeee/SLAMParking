/*
 * DebugWriter.cpp
 *
 *  Created on: Jan 23, 2018
 *      Author: martin
 */

#include "DebugWriter.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

const int START_Y = 25; // y-Value where the initial line goes
const int LINE_HEIGHT = 25; // height of each line
const int POSITION_X = 25; // x-Offset
const Scalar TEST_COLOR = Scalar(0, 0, 255); // Color of the text
const int TEXT_FONT = FONT_HERSHEY_COMPLEX_SMALL; // font of the text
const int TEXT_THICKNESS = 1; // thickness of the text
const double TEXT_HEIGHT = 1; // height of the text

DebugWriter::DebugWriter(Mat &base) {
	this->base = base;
	currentY = START_Y;
}

DebugWriter::~DebugWriter() {
	// nothing to do
}

/**
 * Writes the text and then starts a new line
 */
void DebugWriter::write(std::string str) {
	putText(base, str, Point(POSITION_X, currentY), TEXT_FONT, TEXT_HEIGHT,
			TEST_COLOR, TEXT_THICKNESS);
	currentY += LINE_HEIGHT;
}

/**
 * Resets the line
 */
void DebugWriter::refresh() {
	currentY = START_Y;
}

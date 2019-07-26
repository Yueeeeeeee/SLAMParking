/*
 * VideoProvider.cpp
 *
 *  Created on: May 14, 2018
 *      Author: christian
 */

#include "VideoProvider.h"

cv::Mat VideoProvider::getNextFrame() {
	(*reader) >> currentFrame;
	return currentFrame;
}



VideoProvider::~VideoProvider() {
	delete reader;
}

int VideoProvider::getWidth() {
	return reader->get(cv::CAP_PROP_FRAME_WIDTH);
}

int VideoProvider::getHeight() {
	return reader->get(cv::CAP_PROP_FRAME_HEIGHT);
}

bool VideoProvider::isReady(){
	return reader->isOpened();
}

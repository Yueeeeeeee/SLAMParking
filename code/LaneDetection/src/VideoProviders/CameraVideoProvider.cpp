/*
 * CameraVideoProvider.cpp
 *
 *  Created on: May 14, 2018
 *      Author: christian
 */

#include "CameraVideoProvider.h"

CameraVideoProvider::CameraVideoProvider(int cameraNumber){
	reader = new cv::VideoCapture(cameraNumber);

	// List of supported resolutions
	// v4l2-ctl --list-formats-ext

	// This resolution is only listed for YUYV
	reader->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	reader->set(CV_CAP_PROP_FRAME_HEIGHT, 360);
	reader->set(CV_CAP_PROP_EXPOSURE, 0.5);
}

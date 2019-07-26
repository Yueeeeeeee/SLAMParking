/*
 * VideoDisplay.cpp
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#include "VideoDisplay.h"


VideoDisplay::VideoDisplay(std::string windowName, int width, int height, bool debug) {
	// TODO Auto-generated constructor stub
	this->debug=debug;
	if(debug){
		this->windowName = windowName+" -- Press 'p' for pause/unpause and 's' for single-frame";
	}
	else{
		this->windowName = windowName;
	}



	cv::namedWindow(this->windowName, cv::WINDOW_KEEPRATIO);
	cv::resizeWindow(this->windowName, width, height);
}

VideoDisplay::~VideoDisplay() {
	cv::destroyWindow(this->windowName);
}

void VideoDisplay::consumeFrame(cv::Mat frame) {

	cv::imshow(this->windowName, frame);
	if(this->debug){
		checkUserInput();
	}else{
		cv::waitKey(1);
	}

}

void VideoDisplay::checkUserInput(){
	int key = cv::waitKey(50);
		if (!this->paused) {
			if (key < 0) {
				return;
			} else {
				if (key == 'p') {
					this->paused = true;
				}
			}
		} else {
			while (true) {
			key=cv::waitKey(50);
				if (key == 'p') {
					this->paused = false;
					break;
				}
				else if (key=='s'){
					break;
				}
			}
		}

}

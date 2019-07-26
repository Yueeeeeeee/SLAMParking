/*
 * VideoDisplay.h
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#ifndef SRC_VIDEOCONSUMERS_VIDEODISPLAY_H_
#define SRC_VIDEOCONSUMERS_VIDEODISPLAY_H_

#include "VideoConsumer.h"
#include <opencv2/opencv.hpp>

class VideoDisplay: public VideoConsumer {
public:
	VideoDisplay(std::string windowName, int width, int height,bool debug);
	virtual ~VideoDisplay();
	void consumeFrame(cv::Mat frame);
private:
	void checkUserInput();
	std::string windowName;
	bool paused=false;
	bool debug=false;
};

#endif /* SRC_VIDEOCONSUMERS_VIDEODISPLAY_H_ */

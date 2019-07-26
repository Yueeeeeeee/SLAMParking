/*
 * VideoStreamer.h
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#ifndef SRC_VIDEOCONSUMERS_VIDEOSTREAMER_H_
#define SRC_VIDEOCONSUMERS_VIDEOSTREAMER_H_

#include "VideoConsumer.h"
#include <opencv2/opencv.hpp>
#include <pistache/net.h>
#include <pistache/endpoint.h>
#include <sys/stat.h>

class VideoStreamer: public VideoConsumer {
private:
	std::thread imageServer;
	int current;
public:
	VideoStreamer();
	virtual ~VideoStreamer();
	void consumeFrame(cv::Mat frame);
};

#endif /* SRC_VIDEOCONSUMERS_VIDEOSTREAMER_H_ */

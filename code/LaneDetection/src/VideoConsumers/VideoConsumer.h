/*
 * VideoConsumer.h
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#ifndef SRC_VIDEOCONSUMERS_VIDEOCONSUMER_H_
#define SRC_VIDEOCONSUMERS_VIDEOCONSUMER_H_

#include <opencv2/opencv.hpp>

class VideoConsumer {
public:
	VideoConsumer();
	virtual ~VideoConsumer();
	virtual void consumeFrame(cv::Mat frame)=0;
};

#endif /* SRC_VIDEOCONSUMERS_VIDEOCONSUMER_H_ */

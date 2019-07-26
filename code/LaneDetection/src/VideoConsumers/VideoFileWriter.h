/*
 * VideoFileWriter.h
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#ifndef SRC_VIDEOCONSUMERS_VIDEOFILEWRITER_H_
#define SRC_VIDEOCONSUMERS_VIDEOFILEWRITER_H_

#include "VideoConsumer.h"
#include <opencv2/opencv.hpp>

class VideoFileWriter: public VideoConsumer {
public:
	VideoFileWriter(std::string resultFilename,int width, int height);
	virtual ~VideoFileWriter();
	void consumeFrame(cv::Mat frame);
private:
	cv::VideoWriter* writer;
	int width, height;
};

#endif /* SRC_VIDEOCONSUMERS_VIDEOFILEWRITER_H_ */

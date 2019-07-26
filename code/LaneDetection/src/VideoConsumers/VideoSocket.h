/*
 * VideoSocket.h
 *
 *  Created on: Sep 6, 2018
 *      Author: martin
 */

#ifndef SRC_VIDEOCONSUMERS_VIDEOSOCKET_H_
#define SRC_VIDEOCONSUMERS_VIDEOSOCKET_H_

#include "VideoConsumer.h"
#include <vector>

class VideoSocket: public VideoConsumer {
public:
	VideoSocket();
	virtual ~VideoSocket();
	void consumeFrame(cv::Mat frame);
private:
	static std::vector<int> connections;
	static int sockfd;
	static bool terminationRequest;
	static cv::Mat currentFrame;
	static void setupSocket();
	void destroySocket();
};

#endif /* SRC_VIDEOCONSUMERS_VIDEOSOCKET_H_ */

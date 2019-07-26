#ifndef SRC_VIDEOPROVIDER_H_
#define SRC_VIDEOPROVIDER_H_

#include <opencv2/opencv.hpp>

class VideoProvider{
	public:
	cv::Mat getNextFrame();
	~VideoProvider();
	int getWidth();
	int getHeight();
	bool isReady();

	protected:
	cv::VideoCapture* reader;

	private:
	cv::Mat currentFrame;
};








#endif

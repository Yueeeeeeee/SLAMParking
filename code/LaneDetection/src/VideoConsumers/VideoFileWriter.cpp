/*
 * VideoFileWriter.cpp
 *
 *  Created on: May 15, 2018
 *      Author: christian
 */

#include "VideoFileWriter.h"

VideoFileWriter::VideoFileWriter(std::string resultFilename,int width, int height) {
	// TODO Auto-generated constructor stub
	writer = new cv::VideoWriter(resultFilename, CV_FOURCC('D', 'I', 'V', 'X'),
					30, cv::Size(width, height), true);
	this->width = width;
	this->height = height;

}

VideoFileWriter::~VideoFileWriter() {
	// TODO Auto-generated destructor stub
	delete writer;
}

void VideoFileWriter::consumeFrame(cv::Mat frame){
	cv::Mat tmp;
	cv::resize(frame, tmp, cv::Size(width, height));
	(*writer) << tmp;
}

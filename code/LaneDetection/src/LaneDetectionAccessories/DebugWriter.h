/*
 * DebugWriter.h
 *
 *  Created on: Jan 23, 2018
 *      Author: martin
 */

#ifndef SRC_DEBUGWRITER_H_
#define SRC_DEBUGWRITER_H_

#include <opencv2/opencv.hpp>

/**
 * Allows writing on a Mat
 */
class DebugWriter {
public:
	DebugWriter(cv::Mat &base);
	~DebugWriter();
	void write(std::string str);
	void refresh();
private:
	cv::Mat base;
	int currentY;
};

#endif /* SRC_DEBUGWRITER_H_ */

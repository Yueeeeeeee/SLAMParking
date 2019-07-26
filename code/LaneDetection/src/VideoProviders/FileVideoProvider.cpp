/*
 * FileVideoProvider.cpp
 *
 *  Created on: May 14, 2018
 *      Author: christian
 */

#include "FileVideoProvider.h"

FileVideoProvider::FileVideoProvider(std::string fileName) {
	reader = new cv::VideoCapture(fileName);
}

/*
 * FileVideoProvider.h
 *
 *  Created on: May 14, 2018
 *      Author: christian
 */

#ifndef SRC_FILEVIDEOPROVIDER_H_
#define SRC_FILEVIDEOPROVIDER_H_

#include "VideoProvider.h"


class FileVideoProvider : public VideoProvider {
public:
	FileVideoProvider(std::string fileName);
};

#endif /* SRC_FILEVIDEOPROVIDER_H_ */

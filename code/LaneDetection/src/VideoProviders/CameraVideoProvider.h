/*
 * CameraVideoProvider.h
 *
 *  Created on: May 14, 2018
 *      Author: christian
 */

#ifndef SRC_CAMERAVIDEOPROVIDER_H_
#define SRC_CAMERAVIDEOPROVIDER_H_

#include "VideoProvider.h"

class CameraVideoProvider : public VideoProvider {
public:
	CameraVideoProvider(int cameraNumber);
};

#endif /* SRC_CAMERAVIDEOPROVIDER_H_ */

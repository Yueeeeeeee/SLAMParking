//
// Created by fabian on 15.06.18.
//

#ifndef LANEDETECTION_FRAME_TS_H
#define LANEDETECTION_FRAME_TS_H



#include <mutex>
#include <atomic>
#include "cv.h"

class Frame_ts {
private:
    cv::Mat frame[2];
    int readableFrameIndex = 0;
    mutable std::mutex mutexFrame;
public:
    void setFrame(const cv::Mat &frame);
    const cv::Mat &getFrame() const;
};



#endif //LANEDETECTION_FRAME_TS_H

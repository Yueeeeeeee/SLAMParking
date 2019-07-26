//
// Created by fabian on 15.06.18.
//

#include <opencv2/core/mat.hpp>
#include "Frame_ts.h"



const cv::Mat &Frame_ts::getFrame() const {
    return frame[readableFrameIndex];
}

void Frame_ts::setFrame(const cv::Mat &frame) {
    int newReadableFrameIndex = (readableFrameIndex + 1)%2;

    // This is the critical point. While the new pointer is written no thread can read it.
    std::lock_guard<std::mutex>(this->mutexFrame);
    Frame_ts::frame[newReadableFrameIndex] = frame;

    readableFrameIndex = newReadableFrameIndex;
}

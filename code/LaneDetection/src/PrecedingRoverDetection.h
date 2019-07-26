//
// Created by fabian on 16.06.18.
//

#ifndef LANEDETECTION_PRECEDINGROVERDETECTION_H
#define LANEDETECTION_PRECEDINGROVERDETECTION_H

#include "Frame_ts.h"
#include "PrecedingRover_ts.h"


class PrecedingRoverDetection {
public:
    PrecedingRoverDetection(Frame_ts &currentFrame, PrecedingRover_ts* roverContour);
    void Run();
private:
    Frame_ts* frame;
    PrecedingRover_ts* precedingRover;
    void getContourToErase(cv::Mat &image);
    int framesWithoutRover = 0; // Stores the number of frames without a rover (to smooth the results)
};


#endif //LANEDETECTION_PRECEDINGROVERDETECTION_H

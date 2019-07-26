//
// Created by fabian on 16.06.18.
//

#ifndef LANEDETECTION_ROVERCONTOUR_TS_H
#define LANEDETECTION_ROVERCONTOUR_TS_H


#include <list>
#include <mutex>
#include <opencv2/core/types.hpp>

class PrecedingRover_ts {
private:
    cv::Point *contour[2];
    int readableContourIndex = 0;
    mutable std::mutex mutexContour;

    bool roverRecognized[2];
    int readableRoverRecognized = 0;
    mutable std::mutex mutexRoverRecognized;

    int distance[2];
    int readableDistanceIndex = 0;
    mutable std::mutex mutexDistance;

    int id[2];
    int readableIdIndex = 0;
    mutable std::mutex mutexId;

public:
    PrecedingRover_ts();

    void setContour(cv::Point* contour);
    cv::Point* &getContour();

    void setRoverRecognized(bool roverRecognized);
    bool isRoverRecognized() const;

    void setDistance(int roverRecognized);
    int getDistance() const;

    void setId(int id);
    int getId() const;
};


#endif //LANEDETECTION_ROVERCONTOUR_TS_H

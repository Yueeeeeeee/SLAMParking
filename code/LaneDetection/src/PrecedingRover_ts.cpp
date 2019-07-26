//
// Created by fabian on 16.06.18.
//

#include "PrecedingRover_ts.h"



// Constructor

PrecedingRover_ts::PrecedingRover_ts()
{
    contour[0] = (cv::Point*)malloc(sizeof(cv::Point) * 4);
    contour[1] = (cv::Point*)malloc(sizeof(cv::Point) * 4);
}



// Contour

cv::Point* &PrecedingRover_ts::getContour() {
    std::lock_guard<std::mutex>(this->mutexContour);
    return contour[readableContourIndex];
}

void PrecedingRover_ts::setContour(cv::Point* contour) {
    int newReadableContourIndex = (readableContourIndex + 1)%2;
    PrecedingRover_ts::contour[newReadableContourIndex] = contour;

    // This is the critical point. While the new pointer is written no thread can read it.
    std::lock_guard<std::mutex>(this->mutexContour);
    readableContourIndex = newReadableContourIndex;
}



// RoverRecognized

bool PrecedingRover_ts::isRoverRecognized() const {
    std::lock_guard<std::mutex>(this->mutexContour);
    return roverRecognized[readableRoverRecognized];
}

void PrecedingRover_ts::setRoverRecognized(bool roverRecognized) {
    int newReadableRoverRecognized = (readableRoverRecognized + 1)%2;
    this->roverRecognized[newReadableRoverRecognized] = roverRecognized;

    // This is the critical point. While the new pointer is written no thread can read it.
    std::lock_guard<std::mutex>(this->mutexContour);
    readableRoverRecognized = newReadableRoverRecognized;
}



// Distance

int PrecedingRover_ts::getDistance() const {
    std::lock_guard<std::mutex>(this->mutexDistance);
    return distance[readableDistanceIndex];
}

void PrecedingRover_ts::setDistance(int distance) {
    int newReadableDistanceIndex = (readableDistanceIndex+ 1)%2;
    PrecedingRover_ts::distance[newReadableDistanceIndex] = distance;

    // This is the critical point. While the new pointer is written no thread can read it.
    std::lock_guard<std::mutex>(this->mutexDistance);
    readableDistanceIndex = newReadableDistanceIndex;
}



// Id

int PrecedingRover_ts::getId() const {
    std::lock_guard<std::mutex>(this->mutexId);
    return id[readableIdIndex];
}

void PrecedingRover_ts::setId(int Id) {
    int newReadableIdIndex = (readableIdIndex+ 1)%2;
    PrecedingRover_ts::id[newReadableIdIndex] = Id;

    // This is the critical point. While the new pointer is written no thread can read it.
    std::lock_guard<std::mutex>(this->mutexId);
    readableIdIndex = newReadableIdIndex;
}
//
// Created by janni on 20.05.2019.
//

#ifndef ROVERCTRL_DATA_H
#define ROVERCTRL_DATA_H

#include <stdbool.h>

typedef struct {
    float distanceLeft;
    float distanceRight;
    float yawAngle;
    bool detectedLeft;
    bool detectedRight;
    int precedingRoverDistance; //mm
    int precedingRoverId;
    bool precedingRoverRecognized;
    bool curve;
    float curveRadius;
} LDValues;

typedef struct {
    float distanceLeft;
    float distanceRight;
    float yawAngle;
    bool detectedLeft;
    bool detectedRight;
    bool ldConnected;
    unsigned long msSinceLastUpdate;
} AF3LDValues;

#endif //ROVERCTRL_DATA_H

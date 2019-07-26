//
// Created by Jannik Peters on 11.07.2019.
//

#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <math.h>

#include "vec2f.h"
#include "bezier.h"
#include "pid.h"
#include "velocity_profile.h"

const double EPSILON = 0.00001; // Small number used for floating point comparision.

struct CommandValues
{
    double velocity;
    double steering;
};

class Follower {
private:
    // Configuration parameters
    double wheelbase;
    double steeringLimit;
    double maximumVelocity;
    double acceleration;
    double deceleration;
    double arrivalWindow;
    // Interal state
    PID velocityController;
    CubicBezierCurve spline;
    TrapezoidVelocityProfile profile;
    double tau;
    double distance;
    bool hasArrived;
    bool hasTrajectory;
    Vec2f setpoint;
public:
    Follower(
        double wheelbase,
        double steeringLimit,
        double maximumVelocity,
        double acceleration,
        double deceleration,
        double tvcGainP,
        double tvcTimeI,
        double tvcGainD,
        double arrivalWindow);
    void follow(CubicBezierCurve& curve);
    void abort();
    CommandValues step(double delta_t, double actual_x, double actual_y, double actual_orientation);
    bool arrived();
};

#endif // FOLLOWER_H
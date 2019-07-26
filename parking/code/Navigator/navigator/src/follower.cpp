//
// Created by Jannik Peters on 11.07.2019.
//
#include "follower.h"

Follower::Follower(
        double wheelbase,
        double steeringLimit,
        double maximumVelocity,
        double acceleration,
        double deceleration,
        double tvcGainP,
        double tvcTimeI,
        double tvcGainD,
        double arrivalWindow) : velocityController(tvcGainP, tvcTimeI, tvcGainD) 
{
    this->wheelbase = wheelbase;
    this->steeringLimit = steeringLimit;
    this->arrivalWindow = arrivalWindow;
    this->maximumVelocity = maximumVelocity;
    this->acceleration = acceleration;
    this->deceleration = deceleration;

    this->tau = 0.0;
    this->distance = 0.0;
    this->hasTrajectory = false;
    this->hasArrived = false;

    this->setpoint.x = 0.0;
    this->setpoint.y = 0.0;
}

void Follower::follow(CubicBezierCurve& curve)
{
    this->spline = curve;
    TrapezoidVelocityProfile newprofile(curve.length(), 
        0.66 * this->maximumVelocity, 
        this->acceleration, 
        this->deceleration);
    this->profile = newprofile;
    this->profile.step(sqrt(2.0 * 0.06 / 1.0)); // skip some time
    this->hasTrajectory = true;
    this->hasArrived = false;
    this->tau = 0.0;
    this->distance = 0.0;
}

void Follower::abort()
{
    this->hasTrajectory = false;
    this->hasArrived = false;
}

CommandValues Follower::step(
    double delta_t, 
    double actual_x, 
    double actual_y, 
    double actual_orientation)
{
    if(!arrived()) {
        // Advance on spline
        if(!profile.arrived()) {
            profile.step(delta_t);
            double delta_arc = profile.getDistance() - distance;
            tau = spline.arclength2tau(delta_arc, tau);
            distance = profile.getDistance();
            setpoint = spline.positionAt(tau);
        }

        // calculate position error
        Vec2f actual(actual_x, actual_y);
        Vec2f e = setpoint - actual;
        // Transform to car's local frame.
        e = e.rotate(-actual_orientation);

        // Check arrival
        if(e.length() < arrivalWindow && profile.arrived()) {
            hasArrived = true; // Finish this segment.
        }

        // Command value struct
        CommandValues command;
        double errorDistance;

        // Goal is to the right:
        if (e.y < -EPSILON) {
            double r = (e.y * e.y + e.x * e.x) / (2.0 * -e.y);
            double phi = asin(e.x / r);
            if (2.0 * r < wheelbase) {
                r = wheelbase / 2.0;
            }
            errorDistance = r * phi;
            command.steering = 2.0 * asin(wheelbase / (2.0 * r));
        }
        // Goal is to the left:
        else if (e.y > EPSILON) {
            double r = (e.y * e.y + e.x * e.x) / (2.0 * e.y);
            double phi = asin(e.x / r);
            if (2.0 * r < wheelbase) {
                r = wheelbase / 2.0;
            }
            errorDistance = r * phi;
            command.steering = -2.0 * asin(wheelbase / (2.0 * r));
        }
        // Goal is directly in front.
        else {
            command.steering = 0.0;
            errorDistance = e.x;
        }

        // Use the distance to get a velocity
        command.velocity = velocityController.step(errorDistance, delta_t, maximumVelocity); 

        // Limit the steering angle
        if (command.steering < -steeringLimit) {
            command.steering = -steeringLimit;
        } else if (command.steering > steeringLimit) {
            command.steering = steeringLimit;
        }

        return command;
    }
    else return {0.0, 0.0};
}

bool Follower::arrived() 
{
    return this->hasArrived || !(this->hasTrajectory);
}
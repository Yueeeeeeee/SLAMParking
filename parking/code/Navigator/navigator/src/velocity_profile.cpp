//
// Created by Jannik Peters on 20.05.2019.
//
#include "velocity_profile.h"

TrapezoidVelocityProfile::TrapezoidVelocityProfile() {
    this->totalDistance = 0.0;
    this->travelledDistance = 0.0;
    this->maximumVelocity = 1.0;
    this->currentVelocity = 0.0;
    this->maximumAcceleration = 1.0;
    this->maximumDeceleration = 1.0;
}

TrapezoidVelocityProfile::TrapezoidVelocityProfile(
        double distance,
        double velocity,
        double acceleration,
        double deceleration) {

    this->totalDistance = distance;
    this->travelledDistance = 0.0;
    this->maximumVelocity = velocity;
    this->currentVelocity = 0.0;
    this->maximumAcceleration = acceleration;
    this->maximumDeceleration = deceleration;
}

double TrapezoidVelocityProfile::getBrakingDistance() {
    double t = this->currentVelocity / this->maximumDeceleration;
    return 0.5 * this->maximumDeceleration * t * t;
}

void TrapezoidVelocityProfile::step(double timestep) {
    double remaining = this->totalDistance - this->travelledDistance;
    if(remaining <= 0.0) {
        // Already there. Set velocity to zero to be sure.
        this->currentVelocity = 0.0;
    }
    else if(this->getBrakingDistance() >= remaining) {
        // We need to brake.
        double v = this->currentVelocity - this->maximumDeceleration * timestep;
        if(v < 0.0) {
            v = 0.0;
        }
        this->currentVelocity = v;
    }
    else if(this->currentVelocity < this->maximumVelocity) {
        // We still can go faster.
        double v = this->currentVelocity + this->maximumAcceleration * timestep;
        if(v > this->maximumVelocity) {
            v = this->maximumVelocity;
        }
        this->currentVelocity = v;
    }
    else{
        // Keep maximum velocity.
    }
    this->travelledDistance += this->currentVelocity * timestep;
}

bool TrapezoidVelocityProfile::arrived() {
    return this->travelledDistance >= this->totalDistance;
}

double TrapezoidVelocityProfile::getVelocity() {
    return this->currentVelocity;
}

double TrapezoidVelocityProfile::getDistance() {
    return this->travelledDistance;
}

void TrapezoidVelocityProfile::reset() {
    this->travelledDistance = 0.0;
    this->currentVelocity = 0.0;
}

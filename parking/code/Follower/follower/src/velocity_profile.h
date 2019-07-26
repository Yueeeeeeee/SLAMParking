//
// Created by Jannik Peters on 20.05.2019.
//

#ifndef VELOCITY_PROFILE_H
#define VELOCITY_PROFILE_H

class VelocityProfile {
public:
    /**
     * Calculates the new velocity and travelled distance by advancing a little step in time.
     *
     * @param timestep Time elapsed since the last call in seconds.
     */
    virtual void step(double timestep) = 0;

    /**
     * Gets the set point velocity for the current time step.
     *
     * @return Returns the set point velocity in meters per second.
     */
    virtual double getVelocity() = 0;

    /**
     * Gets the distance travelled so far in meters.
     *
     * @return Returns the travelled distance in meters.
     */
    virtual double getDistance() = 0;

    /**
     * Checks if this velocity profile has finished travelling the target distance.
     *
     * @return Returns true if there is no more distance left to travel.
     */
    virtual bool arrived() = 0;

    /**
     * Resets the travelled distance, so this profile can be reused.
     */
    virtual void reset() = 0;
};

class TrapezoidVelocityProfile : VelocityProfile{
private:
    double totalDistance;
    double travelledDistance;
    double currentVelocity;
    double maximumVelocity;
    double maximumAcceleration;
    double maximumDeceleration;
public:
    TrapezoidVelocityProfile();
    /**
     * Creates a new trapezoid velocity profile. A trapezoid profile will accelerate with constant
     * acceleration until the maximum velocity is reached and will decelerate with constant deceleration to
     * stop right a the target distance.
     *
     * @param distance The overall distance to travel in meters.
     * @param velocity The maximum velocity in meters per second.
     * @param acceleration The acceleration towards maximum velocity in meters per second squared.
     * @param deceleration The deceleration towards standstill in meters per second squared (positive value!).
     */
    TrapezoidVelocityProfile(double distance, double velocity, double acceleration, double deceleration);
    void step(double timestep) override;
    double getVelocity() override;
    double getDistance() override;
    bool arrived() override;
    double getBrakingDistance();
    void reset();
};

#endif //VELOCITY_PROFILE_H

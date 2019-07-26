#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include <signal.h>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>

#include "vector.h"
#include "bezier.h"
#include "velocity_profile.h"
#include "pid.h"

#include <sstream>

// struct for storing setpoint or actual values.
typedef struct {
    float positionX;
    float positionY;
    float orientation;
    float steering;
} values_t;

// constants for the driver state.
const uint8_t STATE_MANUAL = 0;
const uint8_t STATE_PARKING = 1;

// Follower internal constants
const int INPUT_BUFFER_SIZE = 10; // Number of messages in receive buffer
const int OUTPUT_BUFFER_SIZE = 3; // Number of messages in send buffer
const double WHEEL_BASE = 0.32; // [m] distance from rear to front axes of the car.
const double DEFAULT_VELOCITY_LIMIT = 1.2; // [m/s] Maximum allowed velocity in autonomous mode.
const double STEERING_LIMIT = 0.4256; // [rad] = 24.38° Maximum steering angle.
const double EPSILON = 0.00001; // Small number used for floating point comparision.

volatile unsigned char state = STATE_MANUAL; // Current state of the driver node.
volatile bool terminate = false; // Flag used for stopping this node with Ctrl+C
volatile bool goalExists = false; // Flag indicating that there is a meaningfull goal.
volatile values_t actualValues; // Actual values (position, velocity, steering)
volatile values_t setpointValues;
Vec2f goalPosition;
Vec2f goalVector;
// Callback functions for ROS messages
void stateCallback(const std_msgs::UInt8::ConstPtr& msg);
void actualPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void goalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void publishTrajectory(CubicBezierCurve& trajectory, ros::Publisher& trajectoryPublisher);

// Signal handler for handling Ctrl+C
void intHandler(int dummy) 
{
    terminate = true;
}

int main(int argc, char **argv)
{   
    // Init ROS node
    ros::init(argc, argv, "follower");
    ros::NodeHandle n;

    // Subscribe to all required messages
    ros::Subscriber stateSubscriber = n.subscribe("state", INPUT_BUFFER_SIZE, stateCallback);
    ros::Subscriber actualPositionSubscriber = n.subscribe("pose_filtered", INPUT_BUFFER_SIZE, actualPositionCallback);
    ros::Subscriber goalPositionSubscriber = n.subscribe("move_base_simple/goal", INPUT_BUFFER_SIZE, goalPositionCallback);
    // Publish setpoint velocity and steering angle in autonoumous mode.
    ros::Publisher velocityPublisher = n.advertise<std_msgs::Float32>("velocity", OUTPUT_BUFFER_SIZE);
    ros::Publisher steeringAnglePublisher = n.advertise<std_msgs::Float32>("steering", OUTPUT_BUFFER_SIZE);
    ros::Publisher trajectoryPublisher = n.advertise<nav_msgs::Path>("goal_path", 2);
    // Messages
    std_msgs::Float32 velocityMsg;
    std_msgs::Float32 steeringAngleMsg;

    // Controller will run at 100 Hz = 10 ms
    ros::Rate loop_rate(100); // [Hz]
    
    // Curve following
    CubicBezierCurve currentCurve;
    TrapezoidVelocityProfile currentProfile;
    bool hasTrajectory;
    double tau = 0.0;
    double distance = 0.0;

    // PID controller for the velocity.
    double tvcGainP, tvcGainI, tvcGainD;
    n.param<double>("tvc/gains/P", tvcGainP, 0.9);
    n.param<double>("tvc/gains/I", tvcGainI, 60.0);
    n.param<double>("tvc/gains/D", tvcGainD, 0.05);
    PID velocityController(tvcGainP, tvcGainI, tvcGainD);

    // Velocity limit
    double velocityLimit;
    n.param<double>("tvc/limit/velocity", velocityLimit, DEFAULT_VELOCITY_LIMIT);

    double delta_t = 0.01; // [s] time step per loop. See loop_rate.
    unsigned char previousState = STATE_MANUAL; // remember previous driver state to find the rising edge.

    signal(SIGINT, intHandler);

    while(ros::ok() && !terminate) {
        //---------------------------------------------------------------------
        // Manual mode. Do nothing, human is in charge.
        //---------------------------------------------------------------------
        if(state == STATE_MANUAL) {
            velocityMsg.data = 0.0f;
            steeringAngleMsg.data = 0.0f;
            
            velocityPublisher.publish(velocityMsg);
            steeringAnglePublisher.publish(steeringAngleMsg);
        }
        //---------------------------------------------------------------------
        // Path follower and closed loop position controller for auto. Driving.
        //---------------------------------------------------------------------
        else if(state == STATE_PARKING) {
            if(previousState == STATE_MANUAL) {
                // Entering autonomous parking mode.
                if(goalExists) {
                    double veclen = 1.0; // [m]
                    Vec2f p1(actualValues.positionX, actualValues.positionY);
                    Vec2f p2(actualValues.positionX + veclen * sin(actualValues.orientation), actualValues.positionY + veclen * cos(actualValues.orientation));
                    Vec2f p3 = goalPosition - goalVector * veclen;
                    CubicBezierCurve curve(p1, p2, p3, goalPosition);
                    currentCurve = curve;

                    publishTrajectory(curve, trajectoryPublisher);

                    TrapezoidVelocityProfile profile(curve.length(), 0.66 * velocityLimit, 1.0, 1.0);
                    currentProfile = profile;
                    currentProfile.step(sqrt(2.0 * 0.06 / 1.0)); // skip some time

                    hasTrajectory = true;
                    tau = 0.0;
                    distance = 0.0;
                }
            }

            // Calculate the error between setpoint and actual position
            Vec2f e;
            if(hasTrajectory) {
                if(!currentProfile.arrived()) {
                    currentProfile.step(delta_t);
                    double delta_arc = currentProfile.getDistance() - distance;
                    tau = currentCurve.arclength2tau(delta_arc, tau);
                    distance = currentProfile.getDistance();
                    Vec2f currentGoal = currentCurve.positionAt(tau);
                    setpointValues.positionX = currentGoal.x;
                    setpointValues.positionY = currentGoal.y;
                }

                e.x = setpointValues.positionX   - actualValues.positionX;
                e.y = setpointValues.positionY   - actualValues.positionY;
                // Transform to car's local frame.
                e = e.rotate(-actualValues.orientation);

                if(abs(e.x) < 0.05 && abs(e.y) < 0.15 && currentProfile.arrived()) {
                    hasTrajectory = false; // Finish this segment.
                }
            }
            else {
                e.x = 0.0;
                e.y = 0.0;
            }

            double commandSteering;
            double distance;

            // Goal is to the right:
            if (e.y < -EPSILON) {
                double r = (e.y * e.y + e.x * e.x) / (2.0 * -e.y);
                double phi = asin(e.x / r);
                if (2.0 * r < WHEEL_BASE) {
                    r = WHEEL_BASE / 2.0;
                }
                distance = r * phi;
                commandSteering = 2.0 * asin(WHEEL_BASE / (2.0 * r));
            }
            // Goal is to the left:
            else if (e.y > EPSILON) {
                double r = (e.y * e.y + e.x * e.x) / (2.0 * e.y);
                double phi = asin(e.x / r);
                if (2.0 * r < WHEEL_BASE) {
                    r = WHEEL_BASE / 2.0;
                }
                distance = r * phi;
                commandSteering = -2.0 * asin(WHEEL_BASE / (2.0 * r));
            }
            // Goal is directly in front.
            else {
                commandSteering = 0.0;
                distance = e.x;
            }

            // Use the distance to get a velocity
            double commandVelocity = velocityController.step(distance, delta_t, velocityLimit); 

            // Limit the steering angle
            if (commandSteering < -STEERING_LIMIT) {
                commandSteering = -STEERING_LIMIT;
            } else if (commandSteering > STEERING_LIMIT) {
                commandSteering = STEERING_LIMIT;
            }

            // Set messages content.
            velocityMsg.data = (float) commandVelocity;
            steeringAngleMsg.data = (float) commandSteering;
            
            // Publish both messages.
            // TODO: use only one messages including both values.
            velocityPublisher.publish(velocityMsg);
            steeringAnglePublisher.publish(steeringAngleMsg);
        } // End of parking state

        // remember last state
        previousState = state;

        // spin and sleep for up to 10 ms.
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/**
 * Helper function to extract the yaw angle out of a quaternion.
 * The input needs to be a normalized quaternion representing an arbitrary rotation and 
 * the output is the yaw angle in radiant [0..2*pi].
 */
double extractYawFromQuaternion(const geometry_msgs::Quaternion& q) {
    double x = q.x;
    double y = q.y;
    double z = q.z;
    double w = q.w;
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(siny_cosp, cosy_cosp);
    return yaw;
}

/**
 * ROS callback function for the '/state' topic.
 */
void stateCallback(const std_msgs::UInt8::ConstPtr& msg) {
    state = msg->data; // Copy driver state
}

/**
 * ROS callback function for the '/poseupdate' topic. Switch to '/pose_filtered' to use the Kalman filter.
 */
void actualPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // Get position from LIDAR and SLAM.
    actualValues.positionX = msg->pose.pose.position.x;
    actualValues.positionY = msg->pose.pose.position.y;
    // Get orientation from LIDAR and SLAM to ensure consistent frames.
    actualValues.orientation = extractYawFromQuaternion(msg->pose.pose.orientation);

}

/**
 * ROS callback function for the '/move_base_simple/goal' topic.
 * This function sets the controller goal position.
 */
void goalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Goal from Rviz.
    goalExists  = true; //Maybe nice for controller to know this!
    goalPosition.x = msg->pose.position.x;
    goalPosition.y = msg->pose.position.y;
    double orientation = extractYawFromQuaternion(msg->pose.orientation);
    goalVector.x = cos(orientation);
    goalVector.y = sin(orientation);
}

/**
 * Publishes a single cubic bezier curve trajectory as ROS nav_msgs::Path for debugging and 
 * visualization.
 */
void publishTrajectory(CubicBezierCurve& trajectory, ros::Publisher& trajectoryPublisher) {
    nav_msgs::Path msg;

    const int samples = 256;
    std::vector<geometry_msgs::PoseStamped> poses(samples);
    for (int i = 0; i < samples; i++)
    {
        double tau = 1.0 * i / samples;
        Vec2f pos = trajectory.positionAt(tau);
        poses.at(i).pose.position.x = pos.x;
        poses.at(i).pose.position.y = pos.y;
    }
    msg.poses = poses;
    msg.header.frame_id = "map";
    msg.header.time = ros::Time::now();

    trajectoryPublisher.publish(msg);
}


// standard libraries
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
#include <sstream>

// ROS stuff
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>

// our files
#include "vec2f.h"
#include "bezier.h"
#include "follower.h"
#include "planner.h"

// constants for the driver state.
const uint8_t STATE_MANUAL = 0;
const uint8_t STATE_PARKING = 1;

// Follower internal constants
const int INPUT_BUFFER_SIZE = 10; // Number of messages in receive buffer
const int OUTPUT_BUFFER_SIZE = 3; // Number of messages in send buffer
const double WHEEL_BASE = 0.32; // [m] distance from rear to front axes of the car.
const double DEFAULT_VELOCITY_LIMIT = 1.2; // [m/s] Maximum allowed velocity in autonomous mode.
const double STEERING_LIMIT = 0.4256; // [rad] = 24.38° Maximum steering angle.

// actual values
struct {
    double positionX;
    double positionY;
    double orientation;
    unsigned char state = STATE_MANUAL;
} actual;

// goal values
struct {
    double positionX;
    double positionY;
    double orientation;
    bool exists = false;
} goal;

// plan
struct {
    std::vector<CubicBezierCurve> curves;
    bool valid;
    int index;
} path;

volatile bool terminate = false; // Flag used for stopping this node with Ctrl+C

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
    ros::Subscriber actualPositionSubscriber = n.subscribe("poseupdate", INPUT_BUFFER_SIZE, actualPositionCallback);
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

    // PID controller for the velocity.
    double tvcGainP, tvcGainI, tvcGainD;
    n.param<double>("tvc/gains/P", tvcGainP, 0.9);
    n.param<double>("tvc/gains/I", tvcGainI, 60.0);
    n.param<double>("tvc/gains/D", tvcGainD, 0.05);
    PID velocityController(tvcGainP, tvcGainI, tvcGainD);

    ROS_INFO("Starting navigator!");

    // Velocity limit
    double velocityLimit;
    n.param<double>("tvc/limit/velocity", velocityLimit, DEFAULT_VELOCITY_LIMIT);

    // Curve following
    Follower follower(
        WHEEL_BASE,
        STEERING_LIMIT,
        velocityLimit,
        1.0,
        1.0,
        tvcGainP,
        tvcGainI,
        tvcGainD,
        0.08);

    double delta_t = 0.01; // [s] time step per loop. See loop_rate.
    double waiting = 0.0;
    unsigned char previousState = STATE_MANUAL; // remember previous driver state to find the rising edge.

    signal(SIGINT, intHandler);
    CommandValues command;
    command.steering = 0.0;
    command.velocity = 0.0;

    while(ros::ok() && !terminate) {
        //---------------------------------------------------------------------
        // Manual mode. Do nothing, human is in charge.
        //---------------------------------------------------------------------
        if(actual.state == STATE_MANUAL) {
            if(previousState == STATE_PARKING) {
                follower.abort();
            }

            velocityMsg.data = 0.0f;
            steeringAngleMsg.data = 0.0f;
            
            velocityPublisher.publish(velocityMsg);
            steeringAnglePublisher.publish(steeringAngleMsg);
        }
        //---------------------------------------------------------------------
        // Path follower and closed loop position controller for auto. Driving.
        //---------------------------------------------------------------------
        else if(actual.state == STATE_PARKING) {
            if(previousState == STATE_MANUAL) {
                // Entering autonomous parking mode.
                if(goal.exists) {
                    ROS_INFO("Planning path...");
                    Vec2f actualPosition(actual.positionX, actual.positionY);
                    Vec2f actualOrientation(cos(actual.orientation), sin(actual.orientation));
                    Vec2f goalPosition(goal.positionX, goal.positionY);
                    Vec2f goalOrientation(cos(goal.orientation), sin(goal.orientation));
                    path.curves = planPath(actualPosition, actualOrientation, goalPosition, goalOrientation);

                    Vec2f pos = path.curves.at(0).positionAt(0.0);
                    ROS_INFO("Start   = (%f, %f)", pos.x, pos.y);
                    for(int i = 0; i < path.curves.size();i++) {
                        pos = path.curves.at(i).positionAt(1.0);
                        ROS_INFO("Point %d = (%f, %f)", i, pos.x, pos.y);
                    }

                    path.valid = true;
                    path.index = -1;
                }
            }

            if(waiting > 0.0) {
                // keep steering from previous step.
                command.velocity = 0.0;
                waiting -= delta_t;
            }
            else if(path.valid) {
                if(follower.arrived()) {
                    path.index++;
                    if(path.index >= path.curves.size()) {
                        path.valid = false;
                        continue;
                    }
                    else{
                        follower.follow(path.curves.at(path.index));
                        publishTrajectory(path.curves.at(path.index), trajectoryPublisher);
                        waiting = 0.3;
                    }
                }

                command = follower.step(
                    delta_t, 
                    actual.positionX,
                    actual.positionY,
                    actual.orientation);
            }
            else{
                command.velocity = 0.0;
                command.steering = 0.0;
            }

            // Set messages content.
            velocityMsg.data = (float) command.velocity;
            steeringAngleMsg.data = (float) command.steering;
            
            // Publish both messages.
            // TODO: use only one messages including both values.
            velocityPublisher.publish(velocityMsg);
            steeringAnglePublisher.publish(steeringAngleMsg);
        } // End of parking state

        // remember last state
        previousState = actual.state;

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
    actual.state = msg->data; // Copy driver state
}

/**
 * ROS callback function for the '/poseupdate' topic. Switch to '/pose_filtered' to use the Kalman filter.
 */
void actualPositionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // Get position from LIDAR and SLAM.
    actual.positionX = msg->pose.pose.position.x;
    actual.positionY = msg->pose.pose.position.y;
    // Get orientation from LIDAR and SLAM to ensure consistent frames.
    actual.orientation = extractYawFromQuaternion(msg->pose.pose.orientation);

}

/**
 * ROS callback function for the '/move_base_simple/goal' topic.
 * This function sets the controller goal position.
 */
void goalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Goal from Rviz.
    goal.exists  = true; //Maybe nice for controller to know this!
    goal.positionX = msg->pose.position.x;
    goal.positionY = msg->pose.position.y;
    goal.orientation = extractYawFromQuaternion(msg->pose.orientation);
}

/**
 * Publishes a single cubic bezier curve trajectory as ROS nav_msgs::Path for debugging and 
 * visualization.
 */
void publishTrajectory(CubicBezierCurve& trajectory, ros::Publisher& trajectoryPublisher) {
    nav_msgs::Path msg;

    const int samples = 256;
    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(samples);
    auto time = ros::Time::now();
    for (int i = 0; i < samples; i++)
    {
        double tau = (double) i / samples;
        Vec2f pos = trajectory.positionAt(tau);
        geometry_msgs::PoseStamped pose;
        pose.header.seq = i;
        pose.header.stamp = time;
        pose.header.frame_id = "map";
        pose.pose.position.x = pos.x;
        pose.pose.position.y = pos.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0; // For visualization orientation doesn't matter.
        poses.push_back(pose);
    }
    msg.poses = poses;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();

    trajectoryPublisher.publish(msg);
}


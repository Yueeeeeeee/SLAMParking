#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float velocity;
nav_msgs::Odometry odom_msg;
double last_stamp = -1.0;

void velocity_callback(const std_msgs::Float32::ConstPtr& msg)
{
    velocity = msg->data;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double time_delta;
    if (last_stamp == -1.0)
    {
	time_delta = 0.0;
    }
    else
    {
	time_delta = (msg->header.stamp.toSec() - last_stamp);
    }
    last_stamp =  msg->header.stamp.toSec();

    tf2::Quaternion quaternion_orientation;
    tf2::convert(msg->orientation, quaternion_orientation);
    tf2::Quaternion quaternion_velocity = tf2::Quaternion(0, velocity, 0, 0);
    tf2::Quaternion quaternion_rot_vel = quaternion_orientation * quaternion_velocity * quaternion_orientation.inverse();

    odom_msg.pose.pose.orientation = msg->orientation;

    odom_msg.twist.twist.linear.x = quaternion_rot_vel[1];
    odom_msg.twist.twist.linear.y = quaternion_rot_vel[2];
    odom_msg.twist.twist.linear.z = quaternion_rot_vel[3];
    
    odom_msg.twist.twist.angular = msg->angular_velocity;
    
    odom_msg.pose.pose.position.x = quaternion_rot_vel[1] * time_delta;
    odom_msg.pose.pose.position.y = quaternion_rot_vel[2] * time_delta;
    odom_msg.pose.pose.position.z = quaternion_rot_vel[3] * time_delta;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");
    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

    ros::Subscriber velocity_sub = n.subscribe("actual_velocity", 1000, velocity_callback);
    ros::Subscriber imu_sub = n.subscribe("imu", 1000, imu_callback);

    ros::Rate loop_rate(100);

    ros::Time current_time;

    while (ros::ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();

        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
    
        odom_pub.publish(odom_msg);

        loop_rate.sleep();
    }

    return 0;
}

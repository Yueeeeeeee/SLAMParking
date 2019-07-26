#define IPCON_EXPOSE_MILLISLEEP
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include "ip_connection.h"
#include "bricklet_distance_us.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#define HOST "192.168.1.143"
#define PORT 4223
#define UID_RIGHT "DZB"
#define UID_LEFT "zqN" // LEFT Ultrasonic sensor ID: zqN
#define OUTPUT_BUFFER_SIZE 1


volatile bool terminate = false; // Flag used for stopping this node with Ctrl+C
void intHandler(int dummy) 
{
	    terminate = true;
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "brick_ultrasonic");

	ros::NodeHandle n;
	ros::Publisher laserRangePub = n.advertise<std_msgs::Float32>("ultrasonic_range", OUTPUT_BUFFER_SIZE);
	
	// Create IP connection
	IPConnection ipcon;
	ipcon_create(&ipcon);

	// Create device object
	DistanceUS distance_us_right;
	distance_us_create(&distance_us_right, UID_RIGHT, &ipcon);
	DistanceUS distance_us_left;
	distance_us_create(&distance_us_left, UID_LEFT, &ipcon);
	// Connect to brickd
	if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
		fprintf(stderr, "Could not connect\n");
		return 1;
	}
	// Don't use device before ipcon is connected

	ROS_INFO("Connected to brick");
	ros::Rate loop_rate(100);
	uint16_t ultrasonic_distance;
	std_msgs::Float32 distanceMsg;
	float distance;

	signal(SIGINT, intHandler);

	while(ros::ok() && !terminate){
		// right
		if(distance_us_get_distance_value(&distance_us_right, &ultrasonic_distance) < 0) {
			fprintf(stderr, "Could not get distance, probably timeout\n");
	        	return 1;
	   	}

	   	distance = 0.00082 * (float) ultrasonic_distance;
		
		ROS_INFO("Ultrasonic distance: %f", distance); // Not sure about units
		distanceMsg.data = distance;
		laserRangePub.publish(distanceMsg);

		// left
		if(distance_us_get_distance_value(&distance_us_left, &ultrasonic_distance) < 0) {
			fprintf(stderr, "Could not get distance, probably timeout\n");
	        	return 1;
	   	}

	   	distance = 0.00082 * (float) ultrasonic_distance;
		
		ROS_INFO("Ultrasonic distance: %f", distance); // Not sure about units
		distanceMsg.data = distance;
		laserRangePub.publish(distanceMsg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	distance_us_destroy(&distance_us_right);
	distance_us_destroy(&distance_us_left);
	ipcon_destroy(&ipcon); 
	return 0;
}

#define IPCON_EXPOSE_MILLISLEEP
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include "ip_connection.h"
#include "bricklet_laser_range_finder.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#define HOST "192.168.1.143"
#define PORT 4223
#define UID "CQe" // Change XYZ to the UID of your Laser Range Finder Bricklet
#define OUTPUT_BUFFER_SIZE 1


volatile bool terminate = false; // Flag used for stopping this node with Ctrl+C
void intHandler(int dummy) 
{
	    terminate = true;
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "brick_laser_range_finder");

	ros::NodeHandle n;
	ros::Publisher laserRangePub = n.advertise<std_msgs::Float32>("laser_range", OUTPUT_BUFFER_SIZE);
	
	// Create IP connection
	IPConnection ipcon;
	ipcon_create(&ipcon);

	// Create device object
	LaserRangeFinder lrf;
	laser_range_finder_create(&lrf, UID, &ipcon);
	// Connect to brickd
	if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
		fprintf(stderr, "Could not connect\n");
		return 1;
	}
	// Don't use device before ipcon is connected

	// Turn laser on and wait 250ms for very first measurement to be ready
	laser_range_finder_enable_laser(&lrf);
	usleep(250000);
	ROS_INFO("Connected to brick");
	ros::Rate loop_rate(100);
	uint16_t distance_cm;
	float distance_m;
	std_msgs::Float32 distanceMsg;

	signal(SIGINT, intHandler);

	while(ros::ok() && !terminate){
		if(laser_range_finder_get_distance(&lrf, &distance_cm) < 0) {
			fprintf(stderr, "Could not get distance, probably timeout\n");
	        	return 1;
	   	}
		distance_m = (float) distance_cm;
		distance_m = distance_m / 100.0;
		if (distance_m <= 0.02 ){
			distance_m = 40.0;
		}

		ROS_INFO("Distance LASER: %.2f m", distance_m);
		distanceMsg.data = distance_m;
		laserRangePub.publish(distanceMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	laser_range_finder_disable_laser(&lrf); // Turn laser off
	laser_range_finder_destroy(&lrf);
	ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally
	return 0;
}

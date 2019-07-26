#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <signal.h>

#include <stdbool.h>
#include <unistd.h>

extern "C" {
#include "rumblepad.h"
#include "temp_actuator.h"
#include "../subs/vesc/src/hal.h"
#include "../subs/vesc/src/commands.h"
}

#include <pthread.h>


#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"

#include <sstream>

#define L_STICK_ZERO       0
#define L_STICK_MAX    32767
#define L_STICK_MIN   -32768 

#define R2_ZERO            0
#define R2_MIN             0
#define R2_MAX          1024

#define PWM_ZERO        6100
#define PWM_MIN         4600
#define PWM_MAX         7600

#define SPEED_MAX_MS      0.25 //10

// TODO: Replace by a 'gear shift'-function
#define THROTTLE_SPEED_FORWARD  5	
#define THROTTLE_SPEED_BACKWARD 5

#define STEERING_MIN -0.4256
#define STEERING_MAX 0.4256

char* pwm_dev_path = "/dev/ttyACM0"; // Pololu PWM Actuator
char* gp_dev_path  = "/dev/xpadneo"; // Xbox One S Gamepad
char* vesc_path    = "/dev/vesc";    // VESC MCU

static rumblepad_configuration_t* rumblepad_config_1;

volatile bool forward = true;
volatile uint8_t stateManual = true;
const int OUTPUT_BUFFER_SIZE = 3;
const int INPUT_BUFFER_SIZE  = 100;
volatile float velocity;
volatile float steeringAngle;

int steering_angle_pwm = 0;
double steering_angle_norm = 0;
double speed_norm = 0;
double speed_ms = 0;
volatile bool terminate = false;

void intHandler(int dummy) {
    terminate = true;
}

void setVelocity_callback(const std_msgs::Float32::ConstPtr& msg)
{
	if(stateManual == false){
		speed_ms = msg->data;
    		ROS_INFO("Set velocity: %f", msg->data);
	}
}

//TODO: Need to be implemented. Check input values first
void setSteeringAngle_callback(const std_msgs::Float32::ConstPtr& msg)
{
	if(stateManual == false){
		steeringAngle = msg -> data;
		steering_angle_norm = (steeringAngle - STEERING_MIN) / (STEERING_MAX - STEERING_MIN); 
		steering_angle_pwm  = (int) PWM_MIN + steering_angle_norm * (PWM_MAX - PWM_MIN);
		printf("Steering angle: %d", steering_angle_pwm);
        if(steering_angle_pwm > PWM_MAX || steering_angle_pwm < PWM_MIN) {
            steering_angle_pwm = PWM_ZERO;
        }
        temp_actuator_set_target(0, steering_angle_pwm);
		ROS_INFO("Set steering: %f", msg->data);
	}
}

void button_callback (uint16_t button, uint16_t old_value, uint16_t new_value) {

	if (!(new_value == RUMBLEPAD_BUTTON_STATE_PRESSED && old_value == RUMBLEPAD_BUTTON_STATE_NPRESSED))
		return;

	switch (button) {

	case RUMBLEPAD_BUTTON_A:
		forward = false;
		ROS_INFO("action: Backwards\n");
		break;

	case RUMBLEPAD_BUTTON_Y:
		forward = true;
		ROS_INFO("action: Forward\n");
		break;

    case RUMBLEPAD_BUTTON_SELECT:
        if (stateManual == true) {
            stateManual = false;
            ROS_INFO("state: Automatic\n");
        }
        else {
            stateManual = true;
			speed_ms = 0.0;
            ROS_INFO("state: Manual\n");
            }
	}
}



void axis_callback (uint16_t axis, int16_t value) {

	switch (axis) {

	case RUMBLEPAD_AXIS_LEFT_STICK_HORIZONTAL:

		//printf("original value: %i\n", value);
		if(stateManual == true) {
			steering_angle_norm = ((((double) value - L_STICK_MIN ) / (L_STICK_MAX - L_STICK_MIN) ));
			steering_angle_pwm  = PWM_MIN + steering_angle_norm * (PWM_MAX - PWM_MIN);
			steeringAngle = (float) (steering_angle_norm * (STEERING_MAX - STEERING_MIN) + STEERING_MIN);
			temp_actuator_set_target(0, steering_angle_pwm);
        }
		ROS_INFO("normalized steering value: %f\n", steering_angle_norm);
		ROS_INFO("pwm steering value: %i\n", steering_angle_pwm);
		break;

	case RUMBLEPAD_AXIS_R2:
		if(stateManual == true) {
			speed_norm = ((((double) value - R2_MIN ) / (R2_MAX - R2_MIN) ));
			speed_ms   = speed_norm * SPEED_MAX_MS;

			if (!forward)
				speed_ms = -speed_ms;
		}
		ROS_INFO("new speed in m/s: %f\n", speed_ms);
		break;
	}

}

void range_callback(const std_msgs::Float32::ConstPtr& msg)
{
	float epsilon = 0.4;
	float distance = msg->data;
	if (distance < epsilon && !stateManual) {
		stateManual = true;
		speed_ms = 0.0;
	}
}


int main(int argc, char **argv)
{   
    temp_actuator_initialize(pwm_dev_path);
	temp_actuator_set_target(0, PWM_ZERO);
	temp_actuator_set_target(1, PWM_ZERO);
    printf("Connecting to controller "); 
    ROS_INFO("Trying to access RUMBLEPAD at %s ", gp_dev_path);
	fflush(stdout);

	while(access(gp_dev_path, F_OK) == -1) {
		sleep(1);
		printf(".");
		fflush(stdout);
	}
	ROS_INFO("CONNECTED\n");

    rumblepad_config_1 = new rumblepad_configuration_t;
	rumblepad_config_1->device_id = gp_dev_path;
	rumblepad_config_1->waiting_sleep_in_micros = 50;
	rumblepad_config_1->axis_callback = axis_callback;
	rumblepad_config_1->button_callback = button_callback;
	rumblepad_initialize(rumblepad_config_1);
	initUSB(vesc_path, B115200);

    ros::init(argc, argv, "driver");
    ros::NodeHandle n;

    ros::Subscriber velocityRefSubscriber = n.subscribe("velocity", INPUT_BUFFER_SIZE, setVelocity_callback);
	ros::Subscriber steeringAngleSubscriber = n.subscribe("steering", INPUT_BUFFER_SIZE, setSteeringAngle_callback);
	ros::Subscriber range_subscriber = n.subscribe("laser_range", INPUT_BUFFER_SIZE, range_callback);

    ros::Publisher actualVelocity 		= n.advertise<std_msgs::Float32>("actual_velocity", OUTPUT_BUFFER_SIZE);
	ros::Publisher actualSteeringAngle 	= n.advertise<std_msgs::Float32>("actual_steering", OUTPUT_BUFFER_SIZE);
    ros::Publisher statePublisher 		= n.advertise<std_msgs::UInt8>("state", OUTPUT_BUFFER_SIZE);
    
	std_msgs::Float32 velocityFeedbackMsg;
	std_msgs::Float32 steeringAngleFeedbackMsg;
	std_msgs::UInt8 stateMsg;
    ros::Rate loop_rate(100);
    
    signal(SIGINT, intHandler);

	uint8_t stateManualOld = false; 

    while(ros::ok() && !terminate) {
        
        set_velocity(speed_ms);
		//set_angle(steering_angle_pwm);

        ROS_INFO("Velocity feedback: %f ", get_velocity());
        velocityFeedbackMsg.data = (float) get_velocity();
        steeringAngleFeedbackMsg.data =  steeringAngle;
        actualVelocity.publish(velocityFeedbackMsg);
		actualSteeringAngle.publish(steeringAngleFeedbackMsg);

		if (stateManual != stateManualOld) {
			stateMsg.data = stateManual ? 0 : 1;
			statePublisher.publish(stateMsg);
			stateManualOld = stateManual;
		}

        ros::spinOnce();
        loop_rate.sleep();
    }

    delete rumblepad_config_1;

    return 0;
}

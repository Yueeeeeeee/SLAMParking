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
#include <geometry_msgs/Twist.h>

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

#define SPEED_MAX_MS      0.5 //10
#define WHEEL_BASE 0.33

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

double steering_angle = 0;

int steering_angle_pwm = 0;
double steering_angle_norm = 0;
double speed_norm = 0;
double speed_ms = 0;
volatile bool terminate = false;


void intHandler(int dummy) {
    terminate = true;
}

void cmd_vel_callback(const geometry_msgs::Twist msg)
{
	if(stateManual == false){
		steering_angle = -msg.angular.z;
		double velocity = msg.linear.x;

		if (velocity > SPEED_MAX_MS) {
			velocity = SPEED_MAX_MS;
		}
		
		speed_ms = velocity;
		set_velocity(velocity);
   		ROS_INFO("Set velocity: %f", velocity);
	
		steering_angle_norm = (steering_angle - STEERING_MIN) / (STEERING_MAX - STEERING_MIN); 
		steering_angle_pwm  = (int) PWM_MIN + steering_angle_norm * (PWM_MAX - PWM_MIN);
        
		if(steering_angle_pwm > PWM_MAX) {
            steering_angle_pwm = PWM_MAX;
        }
		if(steering_angle_pwm < PWM_MIN) {
        	steering_angle_pwm = PWM_MIN; 
		}

		temp_actuator_set_target(0, steering_angle_pwm);
		ROS_INFO("Set steering: %f", steering_angle);
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
			steering_angle = (double) (steering_angle_norm * (STEERING_MAX - STEERING_MIN) + STEERING_MIN);
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



int main(int argc, char **argv)
{   
    temp_actuator_initialize(pwm_dev_path);
    temp_actuator_set_target(0, PWM_ZERO);
    temp_actuator_set_target(1, PWM_ZERO);
    ROS_INFO("Connecting to controller "); 
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

    ros::init(argc, argv, "nav_stack_driver");
    ros::NodeHandle ros_node;

    ros::Subscriber cmdVelSubscriber = ros_node.subscribe("cmd_vel", INPUT_BUFFER_SIZE, cmd_vel_callback);
    
    ros::Rate loop_rate(100);
    
    signal(SIGINT, intHandler);

    while(ros::ok() && !terminate) {
	set_velocity(speed_ms);
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete rumblepad_config_1;

    return 0;
}

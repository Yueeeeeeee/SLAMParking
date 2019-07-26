// due to current data dictionary declaration of GENTYPE_boolean
// system include must be first
#include <System_ID_4789.h>
#include <timeutil.h>
#include <v2v_message_types.h>
#include <control_center_downstream.h>
#include <camera_client.h>
#include <temp_actuator.h>
#include <hal.h>
#include <v2v_message_handlers.h>
#include <debugprint.h>
#include <rumblepad.h>
#include <af3.h>
#include <v2v_udp_basic.h>
#include <bricklet_distance_us.h>
#include <af3_v2v.h>
#include <canthread.h>
#include <ip_connection.h>
#include <cansocket.h>
#include <bricklet_laser_range_finder.h>
#include <bricklet_rgb_led_button.h>
#include <commands.h>
#include <stdio.h>


#include <stdbool.h>
#include <stddef.h>
#include <unistd.h>

#include <af3.h>
#include <debugprint.h>
#include <timeutil.h>
#include <argp.h>

#define STOP_AT_LOGICAL_CLOCK			(uint64_t)0 // run infinitely

#define BRICK_HOST "localhost"
#define BRICK_PORT 4223

static uint64_t step = 0;
int global_debug_print_level = DEBUG_PRINT_LEVEL_NONE;
const int cycle_time = 33;

//use main arguments 
struct arguments {
	uint64_t local_uid;
};

static struct argp_option options[] = {
		{ "local_uid", 'u', "UID", 0, "Local Rover UID for V2V communications" , 1},
		{ 0 }
};

static error_t parse_opt(int key, char* arg, struct argp_state* state) {
	struct arguments* arguments = state->input;
	switch(key) {

	case 'u':
		arguments->local_uid = atoll(arg);
		break;
	case ARGP_KEY_ARG: return 0;
	default: return ARGP_ERR_UNKNOWN;
	}
	return 0;
}

static char doc[] = "Arguments for V2V communication currently only local uid needed.";
static char args_doc[] = "";
static struct argp argp = { options, parse_opt, args_doc, doc, NULL, NULL, NULL };



static IPConnection brick_connection_singleton_2;
static LaserRangeFinder laser_range_device_instance_3;
static uint16_t laser_range_value_instance_3 = 0;
static uint64_t laser_range_last_cb_time_instance_3 = 0;
static rumblepad_configuration_t* rumblepad_config_singleton_7;
static DistanceUS ultra_sonic_device_instance_9;
static uint16_t ultra_sonic_value_instance_9 = 0;
static uint64_t ultra_sonic_last_cb_time_instance_9 = 0;
static uint16_t rumble_strong_shared_instance_10 = 0;
static uint16_t rumble_weak_shared_instance_10 = 0;
static DistanceUS ultra_sonic_device_instance_11;
static uint16_t ultra_sonic_value_instance_11 = 0;
static uint64_t ultra_sonic_last_cb_time_instance_11 = 0;
RGBLEDButton rgbLEDButton_shared_instance_12;
uint16_t blueValue_shared_instance_12;
RGBLEDButton rgbLEDButton_shared_instance_14;
uint16_t blueValue_shared_instance_14;
uint16_t redValue_shared_instance_14;
uint16_t redValue_shared_instance_12;
static int actuator_pwm_singleton_18 = -1;
uint16_t greenValue_shared_instance_12;
uint16_t greenValue_shared_instance_14;


static void laser_range_callback_instance_3(uint16_t distance, void *data) {
laser_range_value_instance_3 = distance;
laser_range_last_cb_time_instance_3 = time_util_get_current_micros();
}

static void ultra_sonic_callback_instance_9(uint16_t distance, void *data) {
	ultra_sonic_value_instance_9 = distance;
	ultra_sonic_last_cb_time_instance_9 = time_util_get_current_micros();
}

static inline bool ultra_sonic_is_invalid_instance_9() {
	return time_util_get_elapsed_micros_since(ultra_sonic_last_cb_time_instance_9) >= ((uint64_t)1 * SECONDS_IN_MICROS);
}

static void ultra_sonic_callback_instance_11(uint16_t distance, void *data) {
	ultra_sonic_value_instance_11 = distance;
	ultra_sonic_last_cb_time_instance_11 = time_util_get_current_micros();
}

static inline bool ultra_sonic_is_invalid_instance_11() {
	return time_util_get_elapsed_micros_since(ultra_sonic_last_cb_time_instance_11) >= ((uint64_t)1 * SECONDS_IN_MICROS);
}



static uint64_t start_time_in_micros;
static uint64_t delta_time_in_micros;
static uint64_t local_logical_clock;

static void worker() {
	step++;
	noval_LeftStick_X_ID_5201 = false;
	LeftStick_X_ID_5201 = rumblepad_get_axis_position(RUMBLEPAD_AXIS_LEFT_STICK_HORIZONTAL);
	noval_LeftStick_Y_ID_14558 = false;
	LeftStick_Y_ID_14558 = rumblepad_get_axis_position(RUMBLEPAD_AXIS_LEFT_STICK_VERTICAL);
	noval_RightStick_X_ID_6276 = false;
	RightStick_X_ID_6276 = rumblepad_get_axis_position(RUMBLEPAD_AXIS_RIGHT_STICK_HORIZONTAL);
	noval_RightStick_Y_ID_6381 = false;
	RightStick_Y_ID_6381 = rumblepad_get_axis_position(RUMBLEPAD_AXIS_RIGHT_STICK_VERTICAL);
	noval_ButtonL1_ID_5215 = false;
	ButtonL1_ID_5215 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_L1);
	noval_ButtonR1_ID_5229 = false;
	ButtonR1_ID_5229 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_R1);
	noval_ButtonL2_ID_5208 = false;
	ButtonL2_ID_5208 = rumblepad_get_axis_position(RUMBLEPAD_AXIS_L2);
	noval_ButtonR2_ID_5222 = false;
	ButtonR2_ID_5222 = rumblepad_get_axis_position(RUMBLEPAD_AXIS_R2);
	noval_DPadUp_ID_23935 = false;
	DPadUp_ID_23935 = rumblepad_get_button_state(RUMBLEPAD_DPAD_UP);
	noval_DPadDown_ID_23942 = false;
	DPadDown_ID_23942 = rumblepad_get_button_state(RUMBLEPAD_DPAD_DOWN);
	noval_DPadLeft_ID_18559 = false;
	DPadLeft_ID_18559 = rumblepad_get_button_state(RUMBLEPAD_DPAD_LEFT);
	noval_DPadRight_ID_18566 = false;
	DPadRight_ID_18566 = rumblepad_get_button_state(RUMBLEPAD_DPAD_RIGHT);
	noval_ButtonA_ID_5243 = false;
	ButtonA_ID_5243 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_A);
	noval_ButtonB_ID_5250 = false;
	ButtonB_ID_5250 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_B);
	noval_ButtonX_ID_5236 = false;
	ButtonX_ID_5236 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_X);
	noval_ButtonY_ID_5257 = false;
	ButtonY_ID_5257 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_Y);
	noval_ButtonStart_ID_44386 = false;
	ButtonStart_ID_44386 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_START);
	noval_ButtonSelect_ID_14588 = false;
	ButtonSelect_ID_14588 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_SELECT);
	noval_ButtonHome_ID_14595 = false;
	ButtonHome_ID_14595 = rumblepad_get_button_state(RUMBLEPAD_BUTTON_MODE);
	if (!(ultra_sonic_is_invalid_instance_11())) {
	noval_DistanceToFrontUS1_ID_5278 = false;
	DistanceToFrontUS1_ID_5278 = ultra_sonic_value_instance_11;
	} else {
	noval_DistanceToFrontUS1_ID_5278 = true;
	}
	noval_DistanceToFrontLaser_ID_5474 = false;
	DistanceToFrontLaser_ID_5474 = laser_range_value_instance_3;
	noval_VelocityIn_ID_5264 = false;
	VelocityIn_ID_5264 = getVelocity();
	if (!(ultra_sonic_is_invalid_instance_9())) {
	noval_DistanceToFrontUS2_ID_16717 = false;
	DistanceToFrontUS2_ID_16717 = ultra_sonic_value_instance_9;
	} else {
	noval_DistanceToFrontUS2_ID_16717 = true;
	}
	noval_LD_Distance_Left_ID_15718 = false;
	LD_Distance_Left_ID_15718 = camera_client_get_distance_left();
	noval_LD_Distance_Right_ID_15725 = false;
	LD_Distance_Right_ID_15725 = camera_client_get_distance_right();
	noval_LD_present_left_ID_15732 = false;
	LD_present_left_ID_15732 = camera_client_get_detection_state_left();
	noval_LD_present_right_ID_15739 = false;
	LD_present_right_ID_15739 = camera_client_get_detection_state_right();
	noval_LD_Orientation_ID_15746 = false;
	LD_Orientation_ID_15746 = camera_client_get_yaw_angle();
	noval_LD_server_connected_ID_31512 = false;
	LD_server_connected_ID_31512 = camera_client_is_connected();
	noval_LD_server_lastupdate_ID_31505 = false;
	LD_server_lastupdate_ID_31505 = camera_client_get_ms_since_last_update();
	noval_LD_curve_detected_ID_31526 = false;
	LD_curve_detected_ID_31526 = camera_client_is_curve_detected();
	noval_LD_curve_radius_ID_31519 = false;
	LD_curve_radius_ID_31519 = camera_client_get_curve_radius();
	noval_LD_rover_id_ID_31533 = false;
	LD_rover_id_ID_31533 = camera_client_get_preceding_rover_id();
	noval_LD_rover_distance_ID_31540 = false;
	LD_rover_distance_ID_31540 = camera_client_get_preceding_rover_distance();
	noval_LD_rover_detected_ID_31547 = false;
	LD_rover_detected_ID_31547 = camera_client_is_preceding_rover_recognized();
	noval_receivePlatoonMessage_ID_22809 = false;
	af3_v2v_get_platoon( (uint8_t*) &receivePlatoonMessage_ID_22809, &noval_receivePlatoonMessage_ID_22809 , sizeof(receivePlatoonMessage_ID_22809));
	noval_receiveLeaderHB_ID_22823 = false;
	af3_v2v_get_leader_heartbeat( (uint8_t*) &receiveLeaderHB_ID_22823, &noval_receiveLeaderHB_ID_22823 , sizeof(receiveLeaderHB_ID_22823));
	noval_receiveFollowerMessage_ID_22802 = false;
	af3_v2v_get_follower( (uint8_t*) &receiveFollowerMessage_ID_22802, &noval_receiveFollowerMessage_ID_22802 , sizeof(receiveFollowerMessage_ID_22802));
	noval_receiveLeave_ID_22816 = false;
	af3_v2v_get_leave_platoon( (uint8_t*) &receiveLeave_ID_22816, &noval_receiveLeave_ID_22816 , sizeof(receiveLeave_ID_22816));
	noval_receiveSplit_ID_22667 = false;
	af3_v2v_get_split( (uint8_t*) &receiveSplit_ID_22667, &noval_receiveSplit_ID_22667 , sizeof(receiveSplit_ID_22667));
	noval_receiveNewLeader_ID_22788 = false;
	af3_v2v_get_new_leader( (uint8_t*) &receiveNewLeader_ID_22788, &noval_receiveNewLeader_ID_22788 , sizeof(receiveNewLeader_ID_22788));
	noval_receiveFuse_ID_22795 = false;
	af3_v2v_get_fuse_platoon( (uint8_t*) &receiveFuse_ID_22795, &noval_receiveFuse_ID_22795 , sizeof(receiveFuse_ID_22795));
	noval_receiveHandshake_ID_29736 = false;
	af3_v2v_get_handshake( (uint8_t*) &receiveHandshake_ID_29736, &noval_receiveHandshake_ID_29736 , sizeof(receiveHandshake_ID_29736));
	perform_step_System_ID_4789();
	temp_actuator_device_set_target(actuator_pwm_singleton_18, 0, SteeringValue_ID_14149);
	if (noval_MotorValue_ID_7314) {
	;
	} else {
	setVelocity(MotorValue_ID_7314);
	}
	if (noval_RumbleWeak_ID_17086) {
	rumble_weak_shared_instance_10 = 0;
	} else {
	rumble_weak_shared_instance_10 = RumbleWeak_ID_17086;
	}
	if (noval_RumbleStrong_ID_17079) {
	rumble_strong_shared_instance_10 = 0;
	} else {
	rumble_strong_shared_instance_10 = RumbleStrong_ID_17079;
	}
	if (noval_Red_Left_ID_19221) {
	redValue_shared_instance_14 = 0;
	} else {
	redValue_shared_instance_14 = Red_Left_ID_19221;
	}
	if (noval_Green_Left_ID_19228) {
	greenValue_shared_instance_14 = 0;
	} else {
	greenValue_shared_instance_14 = Green_Left_ID_19228;
	}
	if (noval_Blue_Left_ID_19235) {
	blueValue_shared_instance_14 = 0;
	} else {
	blueValue_shared_instance_14 = Blue_Left_ID_19235;
	}
	if (noval_Red_Right_ID_19242) {
	redValue_shared_instance_12 = 0;
	} else {
	redValue_shared_instance_12 = Red_Right_ID_19242;
	}
	if (noval_Green_Right_ID_19249) {
	greenValue_shared_instance_12 = 0;
	} else {
	greenValue_shared_instance_12 = Green_Right_ID_19249;
	}
	if (noval_Blue_Right_ID_19256) {
	blueValue_shared_instance_12 = 0;
	} else {
	blueValue_shared_instance_12 = Blue_Right_ID_19256;
	}
	if (noval_sendFollowerMessage_ID_22712) {
	//Do nothing if noval
	} else {
	af3_v2v_send_msg(V2V_MSG_TYPE_FOLLOWER, (uint8_t*) &sendFollowerMessage_ID_22712, sizeof(sendFollowerMessage_ID_22712) ) ;
	}
	if (noval_sendPlatoonMessage_ID_22719) {
	//Do nothing if noval
	} else {
	af3_v2v_send_msg(V2V_MSG_TYPE_PLATOON, (uint8_t*) &sendPlatoonMessage_ID_22719, sizeof(sendPlatoonMessage_ID_22719) ) ;
	}
	if (noval_sendLeaderHB_ID_22733) {
	//Do nothing if noval
	} else {
	af3_v2v_send_msg(V2V_MSG_TYPE_LEADER_HEARTBEAT, (uint8_t*) &sendLeaderHB_ID_22733, sizeof(sendLeaderHB_ID_22733) ) ;
	}
	if (noval_sendLeave_ID_22726) {
	//Do nothing if noval
	} else {
	af3_v2v_send_msg(V2V_MSG_TYPE_LEAVE_PLATOON, (uint8_t*) &sendLeave_ID_22726, sizeof(sendLeave_ID_22726) ) ;
	}
	if (noval_sendSplit_ID_22691) {
	//Do nothing if noval
	} else {
	af3_v2v_send_msg(V2V_MSG_TYPE_SPLIT, (uint8_t*) &sendSplit_ID_22691, sizeof(sendSplit_ID_22691) ) ;
	}
	if (noval_sendNewLeader_ID_22698) {
	//Do nothing if noval
	} else {
	af3_v2v_send_msg(V2V_MSG_TYPE_NEW_LEADER, (uint8_t*) &sendNewLeader_ID_22698, sizeof(sendNewLeader_ID_22698) ) ;
	}
	if (noval_sendFuse_ID_22705) {
	//Do nothing if noval
	} else {
	af3_v2v_send_msg(V2V_MSG_TYPE_FUSE_PLATOON, (uint8_t*) &sendFuse_ID_22705, sizeof(sendFuse_ID_22705) ) ;
	}
	if (noval_sendHandshake_ID_31148) {
	//Do nothing if noval
	} else {
	af3_v2v_send_msg(V2V_MSG_TYPE_HANDSHAKE, (uint8_t*) &sendHandshake_ID_31148, sizeof(sendHandshake_ID_31148) ) ;
	}
	af3_v2v_set_preceding_vehicle_of_interest_uid(frontVehicleIDtoCodeGen_ID_31126 , noval_frontVehicleIDtoCodeGen_ID_31126 );
	af3_v2v_set_leader_vehicle_of_interest_uid(leaderIDtoCodeGen_ID_31133 , noval_leaderIDtoCodeGen_ID_31133 );
	if (noval_MotorValue_ID_7314) {
	af3_cc_send_noval("MotorValue" , local_logical_clock );
	} else {
	af3_cc_send_double("MotorValue" , MotorValue_ID_7314 , local_logical_clock );
	}
	if (noval_SteeringValue_ID_14149) {
	af3_cc_send_noval("SteeringValue" , local_logical_clock );
	} else {
	af3_cc_send_double("SteeringValue" , SteeringValue_ID_14149 , local_logical_clock );
	}
	if (noval_DistanceFrontOut_ID_11262) {
	af3_cc_send_noval("DistanceFrontOut" , local_logical_clock );
	} else {
	af3_cc_send_double("DistanceFrontOut" , DistanceFrontOut_ID_11262 , local_logical_clock );
	}
	if (noval_VelocityFrontObstacleOut_ID_11255) {
	af3_cc_send_noval("VelocityFrontObstacleOut" , local_logical_clock );
	} else {
	af3_cc_send_double("VelocityFrontObstacleOut" , VelocityFrontObstacleOut_ID_11255 , local_logical_clock );
	}
	if (noval_DistanceToFrontUS_2_ID_44351) {
	af3_cc_send_noval("DistanceToFrontUS_2" , local_logical_clock );
	} else {
	af3_cc_send_double("DistanceToFrontUS_2" , DistanceToFrontUS_2_ID_44351 , local_logical_clock );
	}
	if (noval_DistanceToFrontUS_1_ID_44358) {
	af3_cc_send_noval("DistanceToFrontUS_1" , local_logical_clock );
	} else {
	af3_cc_send_double("DistanceToFrontUS_1" , DistanceToFrontUS_1_ID_44358 , local_logical_clock );
	}
	if (noval_ACC_TargetDistance_ID_44721) {
	af3_cc_send_noval("ACC_TargetDistance" , local_logical_clock );
	} else {
	af3_cc_send_double("ACC_TargetDistance" , ACC_TargetDistance_ID_44721 , local_logical_clock );
	}
	if (noval_ACC_MaxVelocity_ID_44728) {
	af3_cc_send_noval("ACC_MaxVelocity" , local_logical_clock );
	} else {
	af3_cc_send_double("ACC_MaxVelocity" , ACC_MaxVelocity_ID_44728 , local_logical_clock );
	}
	if (noval_ACC_TimeGap_ID_44735) {
	af3_cc_send_noval("ACC_TimeGap" , local_logical_clock );
	} else {
	af3_cc_send_double("ACC_TimeGap" , ACC_TimeGap_ID_44735 , local_logical_clock );
	}
	if (noval_DistanceLaser_ID_44749) {
	af3_cc_send_noval("DistanceLaser" , local_logical_clock );
	} else {
	af3_cc_send_double("DistanceLaser" , DistanceLaser_ID_44749 , local_logical_clock );
	}
	if (noval_Velocity_ID_44756) {
	af3_cc_send_noval("Velocity" , local_logical_clock );
	} else {
	af3_cc_send_double("Velocity" , Velocity_ID_44756 , local_logical_clock );
	}
	if (noval_Critical_ID_44777) {
	af3_cc_send_noval("Critical" , local_logical_clock );
	} else {
	af3_cc_send_boolean("Critical" , Critical_ID_44777 , local_logical_clock );
	}
	if (noval_ForceEBOff_ID_44833) {
	af3_cc_send_noval("ForceEBOff" , local_logical_clock );
	} else {
	af3_cc_send_boolean("ForceEBOff" , ForceEBOff_ID_44833 , local_logical_clock );
	}
	if (noval_LD_Distance_Left_filtered_ID_45381) {
	af3_cc_send_noval("LD_Distance_Left_filtered" , local_logical_clock );
	} else {
	af3_cc_send_double("LD_Distance_Left_filtered" , LD_Distance_Left_filtered_ID_45381 , local_logical_clock );
	}
	if (noval_LD_Distance_Right_filtered_ID_45227) {
	af3_cc_send_noval("LD_Distance_Right_filtered" , local_logical_clock );
	} else {
	af3_cc_send_double("LD_Distance_Right_filtered" , LD_Distance_Right_filtered_ID_45227 , local_logical_clock );
	}
	if (noval_LD_Right_ID_45297) {
	af3_cc_send_noval("LD_Right" , local_logical_clock );
	} else {
	af3_cc_send_double("LD_Right" , LD_Right_ID_45297 , local_logical_clock );
	}
	if (noval_LD_Left_ID_45304) {
	af3_cc_send_noval("LD_Left" , local_logical_clock );
	} else {
	af3_cc_send_double("LD_Left" , LD_Left_ID_45304 , local_logical_clock );
	}
	rumblepad_set_rumble(rumble_strong_shared_instance_10, rumble_weak_shared_instance_10, cycle_time * 0.9, 0);
	rgb_led_button_set_color(&rgbLEDButton_shared_instance_14, redValue_shared_instance_14, greenValue_shared_instance_14, blueValue_shared_instance_14);
	rgb_led_button_set_color(&rgbLEDButton_shared_instance_12, redValue_shared_instance_12, greenValue_shared_instance_12, blueValue_shared_instance_12);

}




int main(int argc, char** argv) {
	
	// set default values of input arguments
	struct arguments arguments;
	arguments.local_uid = -1;
	argp_parse(&argp, argc, argv, 0, 0, &arguments);

    local_logical_clock = 1;

	// initialize the v2v communicaton
	af3_v2v_activate( 9990,  arguments.local_uid, true);
	ipcon_create(&brick_connection_singleton_2);
	if(ipcon_connect(&brick_connection_singleton_2, BRICK_HOST, BRICK_PORT) < 0) {
	perror("Failed to connect to brick sub-system.");
	return 1;
	}

	// initialize LaserRangeSensor
	laser_range_finder_create(&laser_range_device_instance_3, "CQe", &brick_connection_singleton_2);
	laser_range_finder_enable_laser(&laser_range_device_instance_3);
	laser_range_finder_register_callback(&laser_range_device_instance_3, LASER_RANGE_FINDER_CALLBACK_DISTANCE, (void*)laser_range_callback_instance_3, NULL);
	laser_range_finder_set_distance_callback_period(&laser_range_device_instance_3, 10);
	// initialize the control center configuration
	af3_module_initialize("RaspberryPI", 0, NULL, NULL );
	af3_cc_activate("192.168.1.113",9999);

	camera_client_initialize("192.168.21.102", "4444");

	while(access("/dev/input/event0", F_OK) == -1) {
	sleep(1);
	printf("Could not access gamepad device. Trying again in 1s.\n");
	}
	// initialize the rumblepad configuration
	rumblepad_config_singleton_7 = malloc(sizeof(rumblepad_configuration_t));
	rumblepad_config_singleton_7->device_id = "/dev/input/event0";
	rumblepad_config_singleton_7->waiting_sleep_in_micros = 50;
	rumblepad_config_singleton_7->axis_callback = NULL;
	rumblepad_config_singleton_7->button_callback = NULL;
	rumblepad_initialize(rumblepad_config_singleton_7);

	initUSB("/dev/vesc", B115200);
	distance_us_create(&ultra_sonic_device_instance_9, "zr", &brick_connection_singleton_2);
	distance_us_register_callback(&ultra_sonic_device_instance_9, DISTANCE_US_CALLBACK_DISTANCE, (void*)ultra_sonic_callback_instance_9, NULL);
	distance_us_set_distance_callback_period(&ultra_sonic_device_instance_9, 10);
	distance_us_create(&ultra_sonic_device_instance_11, "DZB", &brick_connection_singleton_2);
	distance_us_register_callback(&ultra_sonic_device_instance_11, DISTANCE_US_CALLBACK_DISTANCE, (void*)ultra_sonic_callback_instance_11, NULL);
	distance_us_set_distance_callback_period(&ultra_sonic_device_instance_11, 10);
	rgb_led_button_create(&rgbLEDButton_shared_instance_12, "Dnn", &brick_connection_singleton_2);
	rgb_led_button_create(&rgbLEDButton_shared_instance_14, "Dmb", &brick_connection_singleton_2);
	actuator_pwm_singleton_18 = temp_actuator_initialize("/dev/Maestro0");
	init_System_ID_4789();

	start_time_in_micros = time_util_get_current_micros();
	while(local_logical_clock != STOP_AT_LOGICAL_CLOCK) {
		worker();		
		delta_time_in_micros = time_util_get_elapsed_micros_since(start_time_in_micros);
		if(delta_time_in_micros > cycle_time * MILLIS_IN_MICROS) {
			debug_print(DEBUG_PRINT_LEVEL_FEW, "Remaining cycle time negative! Deadline broken. Shutting down.\n");
			return -1;
		}
		uint64_t remaining_time_in_micros = cycle_time * MILLIS_IN_MICROS - delta_time_in_micros;
		time_util_sleep_micros(remaining_time_in_micros);
		// increase logical clock
		local_logical_clock++;
		// increase start time
		start_time_in_micros += cycle_time * MILLIS_IN_MICROS;
	}
	return 0;
}
/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:08 CEST 2019 */
#include "inc-gen/Steering_Angle_Transformation_ID_5661.h"

GEN_TYPE_double SteeringAngle_ID_5684;

GEN_TYPE_boolean noval_SteeringAngle_ID_5684;

GEN_TYPE_double SteeringValue_ID_20005;

GEN_TYPE_boolean noval_SteeringValue_ID_20005;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_Left_ID_5773();

static GEN_TYPE_boolean fire_transition_Straight_ID_5777();

static GEN_TYPE_boolean fire_transition_Right_ID_5781();

static GEN_TYPE_boolean fire_transition_NoVal_ID_5785();

static GEN_TYPE_boolean fire_transition_NoVal_ID_5789();

static GEN_TYPE_boolean fire_transition_NoVal_ID_5793();

static GEN_TYPE_boolean fire_transition_Straight_ID_5797();

static GEN_TYPE_boolean fire_transition_Left_ID_5801();

static GEN_TYPE_boolean fire_transition_Right_ID_5805();

static GEN_TYPE_boolean fire_transition_Straight_ID_5809();

static GEN_TYPE_boolean fire_transition_Left_ID_5813();

static GEN_TYPE_boolean fire_transition_Right_ID_5817();

static GEN_TYPE_boolean fire_state_Init_NoVal_ID_5757();

static GEN_TYPE_boolean fire_state_Left_ID_5762();

static GEN_TYPE_boolean fire_state_Straight_ID_5765();

static GEN_TYPE_boolean fire_state_Right_ID_5768();

void clear_inputs_Steering_Angle_Transformation_ID_5661(){
	noval_SteeringAngle_ID_5684 = true;
}

void clear_outputs_Steering_Angle_Transformation_ID_5661(){
	noval_SteeringValue_ID_20005 = true;
}

void init_Steering_Angle_Transformation_ID_5661(){
	noval_SteeringValue_ID_20005 = true;
	current_state = 5757;
}

void perform_step_Steering_Angle_Transformation_ID_5661(){
	clear_outputs_Steering_Angle_Transformation_ID_5661();
	if (current_state == 5768) {
		fire_state_Right_ID_5768();
	}
	else {
		if (current_state == 5765) {
			fire_state_Straight_ID_5765();
		}
		else {
			if (current_state == 5762) {
				fire_state_Left_ID_5762();
			}
			else {
				if (current_state == 5757) {
					fire_state_Init_NoVal_ID_5757();
				}
			}
		}
	}
}

static GEN_TYPE_boolean fire_transition_Left_ID_5773(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_LEFT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_LEFT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_LEFT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_LEFT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_LEFT_MAX(), STEERING_ANGLE_LEFT_MIN(), STEERING_VALUE_LEFT_MAX(), STEERING_VALUE_LEFT_MIN());
		
		current_state = 5762;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Straight_ID_5777(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_STRAIGHT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_STRAIGHT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_STRAIGHT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_STRAIGHT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_STRAIGHT_MAX(), STEERING_ANGLE_STRAIGHT_MIN(), STEERING_VALUE_STRAIGHT_MAX(), STEERING_VALUE_STRAIGHT_MIN());
		
		current_state = 5765;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Right_ID_5781(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_RIGHT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_RIGHT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_RIGHT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_RIGHT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_RIGHT_MAX(), STEERING_ANGLE_RIGHT_MIN(), STEERING_VALUE_RIGHT_MAX(), STEERING_VALUE_RIGHT_MIN());
		
		current_state = 5768;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_NoVal_ID_5785(){
	if ((noval_SteeringAngle_ID_5684 == true || SteeringAngle_ID_5684 > STEERING_ANGLE_MAX()) || SteeringAngle_ID_5684 < STEERING_ANGLE_MIN()) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = STEERING_VALUE_STRAIGHT_MIN();
		
		current_state = 5757;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_NoVal_ID_5789(){
	if ((noval_SteeringAngle_ID_5684 == true || SteeringAngle_ID_5684 > STEERING_ANGLE_MAX()) || SteeringAngle_ID_5684 < STEERING_ANGLE_MIN()) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = STEERING_VALUE_STRAIGHT_MIN();
		
		current_state = 5757;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_NoVal_ID_5793(){
	if ((noval_SteeringAngle_ID_5684 == true || SteeringAngle_ID_5684 > STEERING_ANGLE_MAX()) || SteeringAngle_ID_5684 < STEERING_ANGLE_MIN()) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = STEERING_VALUE_STRAIGHT_MIN();
		
		current_state = 5757;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Straight_ID_5797(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_STRAIGHT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_STRAIGHT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_STRAIGHT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_STRAIGHT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_STRAIGHT_MAX(), STEERING_ANGLE_STRAIGHT_MIN(), STEERING_VALUE_STRAIGHT_MAX(), STEERING_VALUE_STRAIGHT_MIN());
		
		current_state = 5765;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Left_ID_5801(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_LEFT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_LEFT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_LEFT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_LEFT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_LEFT_MAX(), STEERING_ANGLE_LEFT_MIN(), STEERING_VALUE_LEFT_MAX(), STEERING_VALUE_LEFT_MIN());
		
		current_state = 5762;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Right_ID_5805(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_RIGHT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_RIGHT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_RIGHT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_RIGHT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_RIGHT_MAX(), STEERING_ANGLE_RIGHT_MIN(), STEERING_VALUE_RIGHT_MAX(), STEERING_VALUE_RIGHT_MIN());
		
		current_state = 5768;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Straight_ID_5809(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_STRAIGHT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_STRAIGHT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_STRAIGHT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_STRAIGHT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_STRAIGHT_MAX(), STEERING_ANGLE_STRAIGHT_MIN(), STEERING_VALUE_STRAIGHT_MAX(), STEERING_VALUE_STRAIGHT_MIN());
		
		current_state = 5765;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Left_ID_5813(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_LEFT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_LEFT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_LEFT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_LEFT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_LEFT_MAX(), STEERING_ANGLE_LEFT_MIN(), STEERING_VALUE_LEFT_MAX(), STEERING_VALUE_LEFT_MIN());
		
		current_state = 5762;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Right_ID_5817(){
	if (!(noval_SteeringAngle_ID_5684 == true) && (SteeringAngle_ID_5684 >= STEERING_ANGLE_RIGHT_MIN() && SteeringAngle_ID_5684 <= STEERING_ANGLE_RIGHT_MAX() || SteeringAngle_ID_5684 <= STEERING_ANGLE_RIGHT_MIN() && SteeringAngle_ID_5684 >= STEERING_ANGLE_RIGHT_MAX())) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_RIGHT_MAX(), STEERING_ANGLE_RIGHT_MIN(), STEERING_VALUE_RIGHT_MAX(), STEERING_VALUE_RIGHT_MIN());
		
		current_state = 5768;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Init_NoVal_ID_5757(){
	if (!(fire_transition_Left_ID_5773() || (fire_transition_Straight_ID_5777() || fire_transition_Right_ID_5781()))) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = STEERING_VALUE_STRAIGHT_MIN();
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Left_ID_5762(){
	if (!(fire_transition_NoVal_ID_5793() || (fire_transition_Straight_ID_5797() || fire_transition_Right_ID_5817()))) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_LEFT_MAX(), STEERING_ANGLE_LEFT_MIN(), STEERING_VALUE_LEFT_MAX(), STEERING_VALUE_LEFT_MIN());
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Straight_ID_5765(){
	if (!(fire_transition_NoVal_ID_5789() || (fire_transition_Left_ID_5801() || fire_transition_Right_ID_5805()))) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_STRAIGHT_MAX(), STEERING_ANGLE_STRAIGHT_MIN(), STEERING_VALUE_STRAIGHT_MAX(), STEERING_VALUE_STRAIGHT_MIN());
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Right_ID_5768(){
	if (!(fire_transition_NoVal_ID_5785() || (fire_transition_Straight_ID_5809() || fire_transition_Left_ID_5813()))) {
		noval_SteeringValue_ID_20005 = false;
		SteeringValue_ID_20005 = linConvert(SteeringAngle_ID_5684, STEERING_ANGLE_RIGHT_MAX(), STEERING_ANGLE_RIGHT_MIN(), STEERING_VALUE_RIGHT_MAX(), STEERING_VALUE_RIGHT_MIN());
		
	}
	return true;
}

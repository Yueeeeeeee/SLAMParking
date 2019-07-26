/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:02 CEST 2019 */
#include "inc-gen/Manual_ID_6500.h"

GEN_TYPE_int GearShift_ID_9872;

GEN_TYPE_boolean noval_GearShift_ID_9872;

GEN_TYPE_double GasPedal_ID_9873;

GEN_TYPE_boolean noval_GasPedal_ID_9873;

GEN_TYPE_double Throttle_ID_9875;

GEN_TYPE_boolean noval_Throttle_ID_9875;

GEN_TYPE_int Gear_ID_12649;

GEN_TYPE_boolean noval_Gear_ID_12649;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_shiftUp_ID_9850();

static GEN_TYPE_boolean fire_transition_shiftDown_ID_9852();

static GEN_TYPE_boolean fire_transition_shiftUp_ID_6567();

static GEN_TYPE_boolean fire_transition_shiftDown_ID_6569();

static GEN_TYPE_boolean fire_transition_shiftUp_ID_9855();

static GEN_TYPE_boolean fire_transition_shiftDown_ID_9857();

static GEN_TYPE_boolean fire_transition_shiftUp_ID_9859();

static GEN_TYPE_boolean fire_transition_shiftDown_ID_9861();

static GEN_TYPE_boolean fire_transition_shiftUp_ID_9863();

static GEN_TYPE_boolean fire_transition_shiftDown_ID_9865();

static GEN_TYPE_boolean fire_transition_reset_ID_6583();

static GEN_TYPE_boolean fire_transition_reset_ID_6585();

static GEN_TYPE_boolean fire_transition_reset_ID_9867();

static GEN_TYPE_boolean fire_transition_reset_ID_9869();

static GEN_TYPE_boolean fire_transition_reset_ID_9871();

static GEN_TYPE_boolean fire_state_Gear1_ID_9812();

static GEN_TYPE_boolean fire_state_Gear2_ID_9819();

static GEN_TYPE_boolean fire_state_Gear3_ID_9824();

static GEN_TYPE_boolean fire_state_Gear4_ID_9831();

static GEN_TYPE_boolean fire_state_Gear5_ID_6545();

static GEN_TYPE_boolean fire_state_Gear6_ID_9843();

void clear_inputs_Manual_ID_6500(){
	noval_GearShift_ID_9872 = true;
	noval_GasPedal_ID_9873 = true;
}

void clear_outputs_Manual_ID_6500(){
	noval_Throttle_ID_9875 = true;
	noval_Gear_ID_12649 = true;
}

void init_Manual_ID_6500(){
	noval_Throttle_ID_9875 = true;
	noval_Gear_ID_12649 = true;
	current_state = 9812;
}

void perform_step_Manual_ID_6500(){
	clear_outputs_Manual_ID_6500();
	if (current_state == 9843) {
		fire_state_Gear6_ID_9843();
	}
	else {
		if (current_state == 6545) {
			fire_state_Gear5_ID_6545();
		}
		else {
			if (current_state == 9831) {
				fire_state_Gear4_ID_9831();
			}
			else {
				if (current_state == 9824) {
					fire_state_Gear3_ID_9824();
				}
				else {
					if (current_state == 9819) {
						fire_state_Gear2_ID_9819();
					}
					else {
						if (current_state == 9812) {
							fire_state_Gear1_ID_9812();
						}
					}
				}
			}
		}
	}
}

static GEN_TYPE_boolean fire_transition_shiftUp_ID_9850(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear2(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 2;
		
		current_state = 9819;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftDown_ID_9852(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == -1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear1(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 1;
		
		current_state = 9812;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftUp_ID_6567(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear3(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 3;
		
		current_state = 9824;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftDown_ID_6569(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == -1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear2(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 2;
		
		current_state = 9819;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftUp_ID_9855(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear4(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 4;
		
		current_state = 9831;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftDown_ID_9857(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == -1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear3(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 3;
		
		current_state = 9824;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftUp_ID_9859(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear5(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 5;
		
		current_state = 6545;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftDown_ID_9861(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == -1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear4(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 4;
		
		current_state = 9831;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftUp_ID_9863(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear6(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 6;
		
		current_state = 9843;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_shiftDown_ID_9865(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == -1) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear5(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 5;
		
		current_state = 6545;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_reset_ID_6583(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 0) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear1(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 1;
		
		current_state = 9812;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_reset_ID_6585(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 0) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear1(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 1;
		
		current_state = 9812;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_reset_ID_9867(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 0) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear1(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 1;
		
		current_state = 9812;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_reset_ID_9869(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 0) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear1(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 1;
		
		current_state = 9812;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_reset_ID_9871(){
	if (noval_GearShift_ID_9872 == false && GearShift_ID_9872 == 0) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear1(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 1;
		
		current_state = 9812;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Gear1_ID_9812(){
	if (!fire_transition_shiftUp_ID_9850()) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear1(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 1;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Gear2_ID_9819(){
	if (!(fire_transition_shiftDown_ID_9852() || (fire_transition_shiftUp_ID_6567() || fire_transition_reset_ID_9867()))) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear2(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 2;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Gear3_ID_9824(){
	if (!(fire_transition_shiftDown_ID_6569() || (fire_transition_shiftUp_ID_9855() || fire_transition_reset_ID_6583()))) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear3(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 3;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Gear4_ID_9831(){
	if (!(fire_transition_shiftDown_ID_9857() || (fire_transition_shiftUp_ID_9859() || fire_transition_reset_ID_9869()))) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear4(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 4;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Gear5_ID_6545(){
	if (!(fire_transition_shiftDown_ID_9861() || (fire_transition_shiftUp_ID_9863() || fire_transition_reset_ID_6585()))) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear5(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 5;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Gear6_ID_9843(){
	if (!(fire_transition_shiftDown_ID_9865() || fire_transition_reset_ID_9871())) {
		noval_Throttle_ID_9875 = false;
		Throttle_ID_9875 = gear6(GasPedal_ID_9873);
		
		noval_Gear_ID_12649 = false;
		Gear_ID_12649 = 6;
		
	}
	return true;
}

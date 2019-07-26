/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:07 CEST 2019 */
#include "inc-gen/R1_ID_13581.h"

GEN_TYPE_int In_ID_13606;

GEN_TYPE_boolean noval_In_ID_13606;

GEN_TYPE_int Out_ID_13604;

GEN_TYPE_boolean noval_Out_ID_13604;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_Press_ID_13600();

static GEN_TYPE_boolean fire_transition_Release_ID_13602();

static GEN_TYPE_boolean fire_state_Released_ID_13588();

static GEN_TYPE_boolean fire_state_Pressed_ID_13593();

void clear_inputs_R1_ID_13581(){
	noval_In_ID_13606 = true;
}

void clear_outputs_R1_ID_13581(){
	noval_Out_ID_13604 = true;
}

void init_R1_ID_13581(){
	noval_Out_ID_13604 = true;
	current_state = 13588;
}

void perform_step_R1_ID_13581(){
	clear_outputs_R1_ID_13581();
	if (current_state == 13593) {
		fire_state_Pressed_ID_13593();
	}
	else {
		if (current_state == 13588) {
			fire_state_Released_ID_13588();
		}
	}
}

static GEN_TYPE_boolean fire_transition_Press_ID_13600(){
	if (noval_In_ID_13606 == false && In_ID_13606 == 1) {
		noval_Out_ID_13604 = false;
		Out_ID_13604 = 1;
		
		current_state = 13593;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Release_ID_13602(){
	if (noval_In_ID_13606 == true || noval_In_ID_13606 == false && In_ID_13606 == 0) {
		noval_Out_ID_13604 = false;
		Out_ID_13604 = 0;
		
		current_state = 13588;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Released_ID_13588(){
	if (!fire_transition_Press_ID_13600()) {
		noval_Out_ID_13604 = true;
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Pressed_ID_13593(){
	if (!fire_transition_Release_ID_13602()) {
		noval_Out_ID_13604 = true;
	}
	return true;
}
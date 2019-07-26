/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:07 CEST 2019 */
#include "inc-gen/DPadDown_ID_19888.h"

GEN_TYPE_int DPadDownIn_ID_32527;

GEN_TYPE_boolean noval_DPadDownIn_ID_32527;

GEN_TYPE_KeyEvent DPadDownOut_ID_32520;

GEN_TYPE_boolean noval_DPadDownOut_ID_32520;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_Press_ID_19930();

static GEN_TYPE_boolean fire_transition_Release_ID_19934();

static GEN_TYPE_boolean fire_state_Released_ID_19920();

static GEN_TYPE_boolean fire_state_Pressed_ID_19925();

void clear_inputs_DPadDown_ID_19888(){
	noval_DPadDownIn_ID_32527 = true;
}

void clear_outputs_DPadDown_ID_19888(){
	noval_DPadDownOut_ID_32520 = true;
}

void init_DPadDown_ID_19888(){
	noval_DPadDownOut_ID_32520 = true;
	current_state = 19920;
}

void perform_step_DPadDown_ID_19888(){
	clear_outputs_DPadDown_ID_19888();
	if (current_state == 19925) {
		fire_state_Pressed_ID_19925();
	}
	else {
		if (current_state == 19920) {
			fire_state_Released_ID_19920();
		}
	}
}

static GEN_TYPE_boolean fire_transition_Press_ID_19930(){
	if (noval_DPadDownIn_ID_32527 == false && DPadDownIn_ID_32527 == 1) {
		noval_DPadDownOut_ID_32520 = false;
		DPadDownOut_ID_32520 = pressed();
		
		current_state = 19925;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Release_ID_19934(){
	if (noval_DPadDownIn_ID_32527 == true || noval_DPadDownIn_ID_32527 == false && DPadDownIn_ID_32527 == 0) {
		noval_DPadDownOut_ID_32520 = false;
		DPadDownOut_ID_32520 = notPressed();
		
		current_state = 19920;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Released_ID_19920(){
	if (!fire_transition_Press_ID_19930()) {
		noval_DPadDownOut_ID_32520 = false;
		DPadDownOut_ID_32520 = notPressed();
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Pressed_ID_19925(){
	if (!fire_transition_Release_ID_19934()) {
		noval_DPadDownOut_ID_32520 = false;
		DPadDownOut_ID_32520 = pressed();
		
	}
	return true;
}

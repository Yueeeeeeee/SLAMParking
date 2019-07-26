/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:02 CEST 2019 */
#include "inc-gen/DPadRight_ID_18732.h"

GEN_TYPE_int DPadRightIn_ID_32532;

GEN_TYPE_boolean noval_DPadRightIn_ID_32532;

GEN_TYPE_KeyEvent DPadRightOut_ID_32515;

GEN_TYPE_boolean noval_DPadRightOut_ID_32515;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_Press_ID_18751();

static GEN_TYPE_boolean fire_transition_Release_ID_18753();

static GEN_TYPE_boolean fire_state_Released_ID_18739();

static GEN_TYPE_boolean fire_state_Pressed_ID_18744();

void clear_inputs_DPadRight_ID_18732(){
	noval_DPadRightIn_ID_32532 = true;
}

void clear_outputs_DPadRight_ID_18732(){
	noval_DPadRightOut_ID_32515 = true;
}

void init_DPadRight_ID_18732(){
	noval_DPadRightOut_ID_32515 = true;
	current_state = 18739;
}

void perform_step_DPadRight_ID_18732(){
	clear_outputs_DPadRight_ID_18732();
	if (current_state == 18744) {
		fire_state_Pressed_ID_18744();
	}
	else {
		if (current_state == 18739) {
			fire_state_Released_ID_18739();
		}
	}
}

static GEN_TYPE_boolean fire_transition_Press_ID_18751(){
	if (noval_DPadRightIn_ID_32532 == false && DPadRightIn_ID_32532 == 1) {
		noval_DPadRightOut_ID_32515 = false;
		DPadRightOut_ID_32515 = pressed();
		
		current_state = 18744;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Release_ID_18753(){
	if (noval_DPadRightIn_ID_32532 == true || noval_DPadRightIn_ID_32532 == false && DPadRightIn_ID_32532 == 0) {
		noval_DPadRightOut_ID_32515 = false;
		DPadRightOut_ID_32515 = notPressed();
		
		current_state = 18739;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Released_ID_18739(){
	if (!fire_transition_Press_ID_18751()) {
		noval_DPadRightOut_ID_32515 = false;
		DPadRightOut_ID_32515 = notPressed();
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Pressed_ID_18744(){
	if (!fire_transition_Release_ID_18753()) {
		noval_DPadRightOut_ID_32515 = false;
		DPadRightOut_ID_32515 = notPressed();
		
	}
	return true;
}

/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:07 CEST 2019 */
#include "inc-gen/Home_ID_13373.h"

GEN_TYPE_int ButtonHomeIn_ID_32562;

GEN_TYPE_boolean noval_ButtonHomeIn_ID_32562;

GEN_TYPE_KeyEvent ButtonHomeOut_ID_32485;

GEN_TYPE_boolean noval_ButtonHomeOut_ID_32485;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_Press_ID_13436();

static GEN_TYPE_boolean fire_transition_Release_ID_13440();

static GEN_TYPE_boolean fire_state_Released_ID_13380();

static GEN_TYPE_boolean fire_state_Pressed_ID_13423();

void clear_inputs_Home_ID_13373(){
	noval_ButtonHomeIn_ID_32562 = true;
}

void clear_outputs_Home_ID_13373(){
	noval_ButtonHomeOut_ID_32485 = true;
}

void init_Home_ID_13373(){
	noval_ButtonHomeOut_ID_32485 = true;
	current_state = 13380;
}

void perform_step_Home_ID_13373(){
	clear_outputs_Home_ID_13373();
	if (current_state == 13423) {
		fire_state_Pressed_ID_13423();
	}
	else {
		if (current_state == 13380) {
			fire_state_Released_ID_13380();
		}
	}
}

static GEN_TYPE_boolean fire_transition_Press_ID_13436(){
	if (noval_ButtonHomeIn_ID_32562 == false && ButtonHomeIn_ID_32562 == 1) {
		noval_ButtonHomeOut_ID_32485 = false;
		ButtonHomeOut_ID_32485 = pressed();
		
		current_state = 13423;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Release_ID_13440(){
	if (noval_ButtonHomeIn_ID_32562 == true || noval_ButtonHomeIn_ID_32562 == false && ButtonHomeIn_ID_32562 == 0) {
		noval_ButtonHomeOut_ID_32485 = false;
		ButtonHomeOut_ID_32485 = notPressed();
		
		current_state = 13380;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Released_ID_13380(){
	if (!fire_transition_Press_ID_13436()) {
		noval_ButtonHomeOut_ID_32485 = false;
		ButtonHomeOut_ID_32485 = notPressed();
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Pressed_ID_13423(){
	if (!fire_transition_Release_ID_13440()) {
		noval_ButtonHomeOut_ID_32485 = false;
		ButtonHomeOut_ID_32485 = pressed();
		
	}
	return true;
}

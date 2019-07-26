/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/L1_ID_13635.h"

GEN_TYPE_int In_ID_13660;

GEN_TYPE_boolean noval_In_ID_13660;

GEN_TYPE_int Out_ID_13658;

GEN_TYPE_boolean noval_Out_ID_13658;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_Press_ID_13654();

static GEN_TYPE_boolean fire_transition_Release_ID_13656();

static GEN_TYPE_boolean fire_state_Released_ID_13642();

static GEN_TYPE_boolean fire_state_Pressed_ID_13647();

void clear_inputs_L1_ID_13635(){
	noval_In_ID_13660 = true;
}

void clear_outputs_L1_ID_13635(){
	noval_Out_ID_13658 = true;
}

void init_L1_ID_13635(){
	noval_Out_ID_13658 = true;
	current_state = 13642;
}

void perform_step_L1_ID_13635(){
	clear_outputs_L1_ID_13635();
	if (current_state == 13647) {
		fire_state_Pressed_ID_13647();
	}
	else {
		if (current_state == 13642) {
			fire_state_Released_ID_13642();
		}
	}
}

static GEN_TYPE_boolean fire_transition_Press_ID_13654(){
	if (noval_In_ID_13660 == false && In_ID_13660 == 1) {
		noval_Out_ID_13658 = false;
		Out_ID_13658 = 1;
		
		current_state = 13647;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Release_ID_13656(){
	if (noval_In_ID_13660 == true || noval_In_ID_13660 == false && In_ID_13660 == 0) {
		noval_Out_ID_13658 = false;
		Out_ID_13658 = 0;
		
		current_state = 13642;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Released_ID_13642(){
	if (!fire_transition_Press_ID_13654()) {
		noval_Out_ID_13658 = true;
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Pressed_ID_13647(){
	if (!fire_transition_Release_ID_13656()) {
		noval_Out_ID_13658 = true;
	}
	return true;
}

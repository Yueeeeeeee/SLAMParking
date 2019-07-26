/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:08 CEST 2019 */
#include "inc-gen/EBOff_ID_17684.h"

GEN_TYPE_KeyEvent ButtonEBOff_ID_17693;

GEN_TYPE_boolean noval_ButtonEBOff_ID_17693;

GEN_TYPE_boolean ForceEBOff_ID_17757;

GEN_TYPE_boolean noval_ForceEBOff_ID_17757;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_Pressed_ID_44946();

static GEN_TYPE_boolean fire_transition_Disabled_ID_44949();

static GEN_TYPE_boolean fire_transition_Pressed_ID_44957();

static GEN_TYPE_boolean fire_transition_Enabled_ID_44961();

static GEN_TYPE_boolean fire_state_EB_enabled_ID_44899();

static GEN_TYPE_boolean fire_state_EB_disabled_ID_44931();

static GEN_TYPE_boolean fire_state_Button_unleashed_ID_44942();

static GEN_TYPE_boolean fire_state_Button_unleashed_ID_44951();

void clear_inputs_EBOff_ID_17684(){
	noval_ButtonEBOff_ID_17693 = true;
}

void clear_outputs_EBOff_ID_17684(){
	noval_ForceEBOff_ID_17757 = true;
}

void init_EBOff_ID_17684(){
	noval_ForceEBOff_ID_17757 = false;
	ForceEBOff_ID_17757 = false;
	
	current_state = 44899;
}

void perform_step_EBOff_ID_17684(){
	clear_outputs_EBOff_ID_17684();
	if (current_state == 44951) {
		fire_state_Button_unleashed_ID_44951();
	}
	else {
		if (current_state == 44942) {
			fire_state_Button_unleashed_ID_44942();
		}
		else {
			if (current_state == 44931) {
				fire_state_EB_disabled_ID_44931();
			}
			else {
				if (current_state == 44899) {
					fire_state_EB_enabled_ID_44899();
				}
			}
		}
	}
}

static GEN_TYPE_boolean fire_transition_Pressed_ID_44946(){
	if (!(noval_ButtonEBOff_ID_17693 == true) && (noval_ButtonEBOff_ID_17693 == false && ButtonEBOff_ID_17693 == pressed())) {
		noval_ForceEBOff_ID_17757 = false;
		ForceEBOff_ID_17757 = false;
		
		current_state = 44942;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Disabled_ID_44949(){
	if (!(noval_ButtonEBOff_ID_17693 == true) && (noval_ButtonEBOff_ID_17693 == false && ButtonEBOff_ID_17693 == notPressed())) {
		noval_ForceEBOff_ID_17757 = false;
		ForceEBOff_ID_17757 = true;
		
		current_state = 44931;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Pressed_ID_44957(){
	if (!(noval_ButtonEBOff_ID_17693 == true) && (noval_ButtonEBOff_ID_17693 == false && ButtonEBOff_ID_17693 == pressed())) {
		noval_ForceEBOff_ID_17757 = false;
		ForceEBOff_ID_17757 = true;
		
		current_state = 44951;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Enabled_ID_44961(){
	if (!(noval_ButtonEBOff_ID_17693 == true) && (noval_ButtonEBOff_ID_17693 == false && ButtonEBOff_ID_17693 == notPressed())) {
		noval_ForceEBOff_ID_17757 = false;
		ForceEBOff_ID_17757 = false;
		
		current_state = 44899;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_EB_enabled_ID_44899(){
	if (!fire_transition_Pressed_ID_44946()) {
		noval_ForceEBOff_ID_17757 = false;
		ForceEBOff_ID_17757 = false;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_EB_disabled_ID_44931(){
	if (!fire_transition_Pressed_ID_44957()) {
		noval_ForceEBOff_ID_17757 = false;
		ForceEBOff_ID_17757 = true;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Button_unleashed_ID_44942(){
	if (!fire_transition_Disabled_ID_44949()) {
		noval_ForceEBOff_ID_17757 = false;
		ForceEBOff_ID_17757 = false;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Button_unleashed_ID_44951(){
	if (!fire_transition_Enabled_ID_44961()) {
		noval_ForceEBOff_ID_17757 = false;
		ForceEBOff_ID_17757 = true;
		
	}
	return true;
}

/* generated by AutoFOCUS 3 (2.14.0) on Fri Jul 05 17:37:26 CEST 2019 */
#include "inc-gen/Platooning_TurnOnOff_ID_20320.h"

GEN_TYPE_AutonomousDrivingEvents ADCommand_ID_20336;

GEN_TYPE_boolean noval_ADCommand_ID_20336;

GEN_TYPE_AD_TurnOffCommands ADTurnOff_ID_27208;

GEN_TYPE_boolean noval_ADTurnOff_ID_27208;

GEN_TYPE_AD_Commands platooning_ID_24124;

GEN_TYPE_boolean noval_platooning_ID_24124;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_AD_On_ID_20358();

static GEN_TYPE_boolean fire_transition_turnoffKeyEvent_ID_27216();

static GEN_TYPE_boolean fire_transition_turnOffAAD_ID_27220();

static GEN_TYPE_boolean fire_transition_turnOffAAD_ID_30120();

static GEN_TYPE_boolean fire_state_Off_EB_ID_20348();

static GEN_TYPE_boolean fire_state_PlatoonMode_ID_20353();

static GEN_TYPE_boolean fire_state_TurnOffInProgress_ID_27211();

void clear_inputs_Platooning_TurnOnOff_ID_20320(){
	noval_ADCommand_ID_20336 = true;
	noval_ADTurnOff_ID_27208 = true;
}

void clear_outputs_Platooning_TurnOnOff_ID_20320(){
	noval_platooning_ID_24124 = true;
}

void init_Platooning_TurnOnOff_ID_20320(){
	noval_platooning_ID_24124 = true;
	current_state = 20348;
}

void perform_step_Platooning_TurnOnOff_ID_20320(){
	clear_outputs_Platooning_TurnOnOff_ID_20320();
	if (current_state == 27211) {
		fire_state_TurnOffInProgress_ID_27211();
	}
	else {
		if (current_state == 20353) {
			fire_state_PlatoonMode_ID_20353();
		}
		else {
			if (current_state == 20348) {
				fire_state_Off_EB_ID_20348();
			}
		}
	}
}

static GEN_TYPE_boolean fire_transition_AD_On_ID_20358(){
	if (!(noval_ADCommand_ID_20336 == true) && ADCommand_ID_20336.platooning == pressed()) {
		noval_platooning_ID_24124 = false;
		platooning_ID_24124 = OnAD();
		
		current_state = 20353;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_turnoffKeyEvent_ID_27216(){
	if (!(noval_ADCommand_ID_20336 == true) && ADCommand_ID_20336.platooning == pressed()) {
		noval_platooning_ID_24124 = false;
		platooning_ID_24124 = OffAD();
		
		current_state = 27211;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_turnOffAAD_ID_27220(){
	if (!(noval_ADTurnOff_ID_27208 == true) && ADTurnOff_ID_27208.Platooning == true) {
		current_state = 20348;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_turnOffAAD_ID_30120(){
	if (!(noval_ADTurnOff_ID_27208 == true) && ADTurnOff_ID_27208.Platooning == true) {
		current_state = 20348;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Off_EB_ID_20348(){
	if (!fire_transition_AD_On_ID_20358()) {
		noval_platooning_ID_24124 = false;
		platooning_ID_24124 = KeepCurrentStateAD();
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_PlatoonMode_ID_20353(){
	if (!(fire_transition_turnoffKeyEvent_ID_27216() || fire_transition_turnOffAAD_ID_30120())) {
		noval_platooning_ID_24124 = false;
		platooning_ID_24124 = KeepCurrentStateAD();
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_TurnOffInProgress_ID_27211(){
	if (!fire_transition_turnOffAAD_ID_27220()) {
		noval_platooning_ID_24124 = false;
		platooning_ID_24124 = KeepCurrentStateAD();
		
	}
	return true;
}

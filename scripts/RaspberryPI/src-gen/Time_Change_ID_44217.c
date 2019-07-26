/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:03 CEST 2019 */
#include "inc-gen/Time_Change_ID_44217.h"

GEN_TYPE_DriverAssistanceCommands DA_States_ID_53829;

GEN_TYPE_boolean noval_DA_States_ID_53829;

GEN_TYPE_int TimeChange_ID_44288;

GEN_TYPE_boolean noval_TimeChange_ID_44288;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_Rising_flank_ID_44243();

static GEN_TYPE_boolean fire_transition_Falling_flank_ID_44246();

static GEN_TYPE_boolean fire_transition_Rising_flank_ID_44249();

static GEN_TYPE_boolean fire_transition_Falling_flank_ID_44252();

static GEN_TYPE_boolean fire_state_Released_ID_44224();

static GEN_TYPE_boolean fire_state_Plus__Still_pushed_ID_44231();

static GEN_TYPE_boolean fire_state_Minus__Still_pushed_ID_44236();

void clear_inputs_Time_Change_ID_44217(){
	noval_DA_States_ID_53829 = true;
}

void clear_outputs_Time_Change_ID_44217(){
	noval_TimeChange_ID_44288 = true;
}

void init_Time_Change_ID_44217(){
	noval_TimeChange_ID_44288 = true;
	current_state = 44224;
}

void perform_step_Time_Change_ID_44217(){
	clear_outputs_Time_Change_ID_44217();
	if (current_state == 44236) {
		fire_state_Minus__Still_pushed_ID_44236();
	}
	else {
		if (current_state == 44231) {
			fire_state_Plus__Still_pushed_ID_44231();
		}
		else {
			if (current_state == 44224) {
				fire_state_Released_ID_44224();
			}
		}
	}
}

static GEN_TYPE_boolean fire_transition_Rising_flank_ID_44243(){
	if (!(noval_DA_States_ID_53829 == true) && DA_States_ID_53829.timePlus == pressed()) {
		noval_TimeChange_ID_44288 = false;
		TimeChange_ID_44288 = 1;
		
		current_state = 44231;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Falling_flank_ID_44246(){
	if (noval_DA_States_ID_53829 == true || DA_States_ID_53829.timePlus == notPressed()) {
		noval_TimeChange_ID_44288 = false;
		TimeChange_ID_44288 = 0;
		
		current_state = 44224;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Rising_flank_ID_44249(){
	if (!(noval_DA_States_ID_53829 == true) && DA_States_ID_53829.timeMinus == pressed()) {
		noval_TimeChange_ID_44288 = false;
		TimeChange_ID_44288 = -1;
		
		current_state = 44236;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_Falling_flank_ID_44252(){
	if (noval_DA_States_ID_53829 == true || DA_States_ID_53829.timeMinus == notPressed()) {
		noval_TimeChange_ID_44288 = false;
		TimeChange_ID_44288 = 0;
		
		current_state = 44224;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_Released_ID_44224(){
	if (!(fire_transition_Rising_flank_ID_44243() || fire_transition_Rising_flank_ID_44249())) {
		noval_TimeChange_ID_44288 = false;
		TimeChange_ID_44288 = 0;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Plus__Still_pushed_ID_44231(){
	if (!fire_transition_Falling_flank_ID_44246()) {
		noval_TimeChange_ID_44288 = false;
		TimeChange_ID_44288 = 0;
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_Minus__Still_pushed_ID_44236(){
	if (!fire_transition_Falling_flank_ID_44252()) {
		noval_TimeChange_ID_44288 = false;
		TimeChange_ID_44288 = 0;
		
	}
	return true;
}

/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/Prevent_cycle_ID_10366.h"

GEN_TYPE_DA_TurnOff_Commands DA_TurnOff_In_ID_16213;

GEN_TYPE_boolean noval_DA_TurnOff_In_ID_16213;

GEN_TYPE_DA_TurnOff_Commands DA_TurnOff_Out_ID_10382;

GEN_TYPE_boolean noval_DA_TurnOff_Out_ID_10382;

void clear_inputs_Prevent_cycle_ID_10366(){
	noval_DA_TurnOff_In_ID_16213 = true;
}

void clear_outputs_Prevent_cycle_ID_10366(){
	noval_DA_TurnOff_Out_ID_10382 = true;
}

void init_Prevent_cycle_ID_10366(){
	noval_DA_TurnOff_Out_ID_10382 = true;
}

void perform_step_Prevent_cycle_ID_10366(){
	clear_outputs_Prevent_cycle_ID_10366();
	noval_DA_TurnOff_Out_ID_10382 = noval_DA_TurnOff_In_ID_16213;
	if (!noval_DA_TurnOff_In_ID_16213) {
		DA_TurnOff_Out_ID_10382 = DA_TurnOff_In_ID_16213;
	}
	
	return;
	
}

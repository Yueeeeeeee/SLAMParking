/* generated by AutoFOCUS 3 (2.14.0) on Fri Jul 05 17:37:26 CEST 2019 */
#include "inc-gen/Weight_Current_ID_45058.h"

GEN_TYPE_double Weight_Current_ID_45064;

GEN_TYPE_boolean noval_Weight_Current_ID_45064;

void clear_inputs_Weight_Current_ID_45058(){
}

void clear_outputs_Weight_Current_ID_45058(){
	noval_Weight_Current_ID_45064 = true;
}

void init_Weight_Current_ID_45058(){
	noval_Weight_Current_ID_45064 = true;
}

void perform_step_Weight_Current_ID_45058(){
	clear_outputs_Weight_Current_ID_45058();
	noval_Weight_Current_ID_45064 = false;
	Weight_Current_ID_45064 = LK_FILTER_WEIGHT();
	
	return;
	
}
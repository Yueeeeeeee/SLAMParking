/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:03 CEST 2019 */
#include "inc-gen/Weight_Current_ID_73.h"

GEN_TYPE_double Weight_Current_ID_45027;

GEN_TYPE_boolean noval_Weight_Current_ID_45027;

void clear_inputs_Weight_Current_ID_73(){
}

void clear_outputs_Weight_Current_ID_73(){
	noval_Weight_Current_ID_45027 = true;
}

void init_Weight_Current_ID_73(){
	noval_Weight_Current_ID_45027 = true;
}

void perform_step_Weight_Current_ID_73(){
	clear_outputs_Weight_Current_ID_73();
	noval_Weight_Current_ID_45027 = false;
	Weight_Current_ID_45027 = LK_FILTER_WEIGHT();
	
	return;
	
}

/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:08 CEST 2019 */
#include "inc-gen/Computation_ID_45034.h"

GEN_TYPE_double History_ID_45042;

GEN_TYPE_boolean noval_History_ID_45042;

GEN_TYPE_double Input_ID_45044;

GEN_TYPE_boolean noval_Input_ID_45044;

GEN_TYPE_double Weight_Current_ID_45046;

GEN_TYPE_boolean noval_Weight_Current_ID_45046;

GEN_TYPE_double Output_ID_45040;

GEN_TYPE_boolean noval_Output_ID_45040;

void clear_inputs_Computation_ID_45034(){
	noval_History_ID_45042 = true;
	noval_Input_ID_45044 = true;
	noval_Weight_Current_ID_45046 = true;
}

void clear_outputs_Computation_ID_45034(){
	noval_Output_ID_45040 = true;
}

void init_Computation_ID_45034(){
	noval_Output_ID_45040 = true;
}

void perform_step_Computation_ID_45034(){
	clear_outputs_Computation_ID_45034();
	noval_Output_ID_45040 = false;
	Output_ID_45040 = Weight_Current_ID_45046 * Input_ID_45044 + (1 - Weight_Current_ID_45046) * History_ID_45042;
	
	return;
	
}

/* generated by AutoFOCUS 3 (2.14.0) on Fri Jul 05 17:37:26 CEST 2019 */
#include "inc-gen/Mem_ID_12576.h"

GEN_TYPE_DriveMode StateNew_ID_12587;

GEN_TYPE_boolean noval_StateNew_ID_12587;

GEN_TYPE_DriveMode StateOld_ID_12594;

GEN_TYPE_boolean noval_StateOld_ID_12594;

void clear_inputs_Mem_ID_12576(){
	noval_StateNew_ID_12587 = true;
}

void clear_outputs_Mem_ID_12576(){
	noval_StateOld_ID_12594 = true;
}

void init_Mem_ID_12576(){
	noval_StateOld_ID_12594 = true;
}

void perform_step_Mem_ID_12576(){
	clear_outputs_Mem_ID_12576();
	noval_StateOld_ID_12594 = noval_StateNew_ID_12587;
	if (!noval_StateNew_ID_12587) {
		StateOld_ID_12594 = StateNew_ID_12587;
	}
	
	return;
	
}

/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/Delay_ID_13898.h"

GEN_TYPE_double MaxAccelerationEB_ID_29303;

GEN_TYPE_boolean noval_MaxAccelerationEB_ID_29303;

GEN_TYPE_boolean EB_On_ID_14053;

GEN_TYPE_boolean noval_EB_On_ID_14053;

void clear_inputs_Delay_ID_13898(){
	noval_MaxAccelerationEB_ID_29303 = true;
}

void clear_outputs_Delay_ID_13898(){
	noval_EB_On_ID_14053 = true;
}

void init_Delay_ID_13898(){
	noval_EB_On_ID_14053 = true;
}

void perform_step_Delay_ID_13898(){
	clear_outputs_Delay_ID_13898();
	if (!(noval_MaxAccelerationEB_ID_29303 == true)) {
		noval_EB_On_ID_14053 = false;
		EB_On_ID_14053 = true;
		
	}
	else {
		noval_EB_On_ID_14053 = false;
		EB_On_ID_14053 = false;
		
	}
	return;
	
}
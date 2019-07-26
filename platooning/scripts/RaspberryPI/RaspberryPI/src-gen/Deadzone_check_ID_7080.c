/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:08 CEST 2019 */
#include "inc-gen/Deadzone_check_ID_7080.h"

GEN_TYPE_double TargetVelocity_ID_19712;

GEN_TYPE_boolean noval_TargetVelocity_ID_19712;

GEN_TYPE_double OutputValue_ID_7186;

GEN_TYPE_boolean noval_OutputValue_ID_7186;

void clear_inputs_Deadzone_check_ID_7080(){
	noval_TargetVelocity_ID_19712 = true;
}

void clear_outputs_Deadzone_check_ID_7080(){
	noval_OutputValue_ID_7186 = true;
}

void init_Deadzone_check_ID_7080(){
	noval_OutputValue_ID_7186 = true;
}

void perform_step_Deadzone_check_ID_7080(){
	clear_outputs_Deadzone_check_ID_7080();
	if (noval_TargetVelocity_ID_19712 == true) {
		noval_OutputValue_ID_7186 = false;
		OutputValue_ID_7186 = 0;
		
		return;
	}
	if (myabs(TargetVelocity_ID_19712) < VELOCITY_DEADZONE()) {
		noval_OutputValue_ID_7186 = false;
		OutputValue_ID_7186 = 0.0;
		
	}
	else {
		noval_OutputValue_ID_7186 = noval_TargetVelocity_ID_19712;
		if (!noval_TargetVelocity_ID_19712) {
			OutputValue_ID_7186 = TargetVelocity_ID_19712;
		}
		
	}
	return;
	
}

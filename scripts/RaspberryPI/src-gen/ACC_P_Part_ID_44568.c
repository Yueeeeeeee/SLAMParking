/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:02 CEST 2019 */
#include "inc-gen/ACC_P_Part_ID_44568.h"

GEN_TYPE_double Error_adjusted_ID_44643;

GEN_TYPE_boolean noval_Error_adjusted_ID_44643;

GEN_TYPE_double Velocity_ID_44616;

GEN_TYPE_boolean noval_Velocity_ID_44616;

GEN_TYPE_double TargetVelocity_ID_44598;

GEN_TYPE_boolean noval_TargetVelocity_ID_44598;

void clear_inputs_ACC_P_Part_ID_44568(){
	noval_Error_adjusted_ID_44643 = true;
	noval_Velocity_ID_44616 = true;
}

void clear_outputs_ACC_P_Part_ID_44568(){
	noval_TargetVelocity_ID_44598 = true;
}

void init_ACC_P_Part_ID_44568(){
	noval_TargetVelocity_ID_44598 = true;
}

void perform_step_ACC_P_Part_ID_44568(){
	clear_outputs_ACC_P_Part_ID_44568();
	if (!(noval_Error_adjusted_ID_44643 == true) && !(noval_Velocity_ID_44616 == true)) {
		noval_TargetVelocity_ID_44598 = false;
		TargetVelocity_ID_44598 = max(0.0, (Velocity_ID_44616 + Error_adjusted_ID_44643 * ACC_CONTROLLER_P()));
		
	}
	else {
		noval_TargetVelocity_ID_44598 = false;
		TargetVelocity_ID_44598 = 0.0;
		
	}
	return;
	
}

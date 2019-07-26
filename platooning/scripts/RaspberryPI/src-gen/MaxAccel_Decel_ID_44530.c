/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/MaxAccel_Decel_ID_44530.h"

GEN_TYPE_double TargetVelocityOut_ID_44539;

GEN_TYPE_boolean noval_TargetVelocityOut_ID_44539;

GEN_TYPE_double Velocity_ID_44561;

GEN_TYPE_boolean noval_Velocity_ID_44561;

GEN_TYPE_double TargetVelocity_ID_44544;

GEN_TYPE_boolean noval_TargetVelocity_ID_44544;

GEN_TYPE_double MaxAcceleration_ID_44551;

GEN_TYPE_boolean noval_MaxAcceleration_ID_44551;

void clear_inputs_MaxAccel_Decel_ID_44530(){
	noval_TargetVelocityOut_ID_44539 = true;
	noval_Velocity_ID_44561 = true;
}

void clear_outputs_MaxAccel_Decel_ID_44530(){
	noval_TargetVelocity_ID_44544 = true;
	noval_MaxAcceleration_ID_44551 = true;
}

void init_MaxAccel_Decel_ID_44530(){
	noval_TargetVelocity_ID_44544 = true;
	noval_MaxAcceleration_ID_44551 = true;
}

void perform_step_MaxAccel_Decel_ID_44530(){
	clear_outputs_MaxAccel_Decel_ID_44530();
	if (!(noval_TargetVelocityOut_ID_44539 == true) && !(noval_Velocity_ID_44561 == true)) {
		noval_TargetVelocity_ID_44544 = noval_TargetVelocityOut_ID_44539;
		if (!noval_TargetVelocityOut_ID_44539) {
			TargetVelocity_ID_44544 = TargetVelocityOut_ID_44539;
		}
		
		if (TargetVelocityOut_ID_44539 < Velocity_ID_44561) {
			noval_MaxAcceleration_ID_44551 = false;
			MaxAcceleration_ID_44551 = ACC_MAX_DECELERATION();
			
		}
		else {
			noval_MaxAcceleration_ID_44551 = false;
			MaxAcceleration_ID_44551 = ACC_MAX_ACCELERATION();
			
		}
	}
	else {
		noval_TargetVelocity_ID_44544 = true;
		noval_MaxAcceleration_ID_44551 = true;
	}
	return;
	
}

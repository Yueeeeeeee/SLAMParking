/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:02 CEST 2019 */
#include "inc-gen/Boundaries_ID_16309.h"

GEN_TYPE_double SteeringAngleRegulated_In_ID_16318;

GEN_TYPE_boolean noval_SteeringAngleRegulated_In_ID_16318;

GEN_TYPE_double SteeringAngleRegulated_ID_16323;

GEN_TYPE_boolean noval_SteeringAngleRegulated_ID_16323;

GEN_TYPE_boolean BoundariesActive_ID_16345;

GEN_TYPE_boolean noval_BoundariesActive_ID_16345;

void clear_inputs_Boundaries_ID_16309(){
	noval_SteeringAngleRegulated_In_ID_16318 = true;
}

void clear_outputs_Boundaries_ID_16309(){
	noval_SteeringAngleRegulated_ID_16323 = true;
	noval_BoundariesActive_ID_16345 = true;
}

void init_Boundaries_ID_16309(){
	noval_SteeringAngleRegulated_ID_16323 = true;
	noval_BoundariesActive_ID_16345 = true;
}

void perform_step_Boundaries_ID_16309(){
	clear_outputs_Boundaries_ID_16309();
	if (SteeringAngleRegulated_In_ID_16318 > STEERING_ANGLE_MAX()) {
		noval_BoundariesActive_ID_16345 = false;
		BoundariesActive_ID_16345 = true;
		
		noval_SteeringAngleRegulated_ID_16323 = false;
		SteeringAngleRegulated_ID_16323 = STEERING_ANGLE_MAX();
		
	}
	else {
		if (myabs(SteeringAngleRegulated_In_ID_16318) > STEERING_ANGLE_MAX()) {
			noval_BoundariesActive_ID_16345 = false;
			BoundariesActive_ID_16345 = true;
			
			noval_SteeringAngleRegulated_ID_16323 = false;
			SteeringAngleRegulated_ID_16323 = -STEERING_ANGLE_MAX();
			
		}
		else {
			noval_BoundariesActive_ID_16345 = false;
			BoundariesActive_ID_16345 = false;
			
			noval_SteeringAngleRegulated_ID_16323 = noval_SteeringAngleRegulated_In_ID_16318;
			if (!noval_SteeringAngleRegulated_In_ID_16318) {
				SteeringAngleRegulated_ID_16323 = SteeringAngleRegulated_In_ID_16318;
			}
			
		}
	}
	return;
	
}

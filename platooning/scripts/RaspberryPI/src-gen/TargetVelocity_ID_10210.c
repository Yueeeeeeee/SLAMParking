/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/TargetVelocity_ID_10210.h"

GEN_TYPE_double MaxVelocity_ID_10230;

GEN_TYPE_boolean noval_MaxVelocity_ID_10230;

GEN_TYPE_double TargetVelocityIn_ID_16462;

GEN_TYPE_boolean noval_TargetVelocityIn_ID_16462;

GEN_TYPE_DriverAssistanceCommands DA_States_ID_53839;

GEN_TYPE_boolean noval_DA_States_ID_53839;

GEN_TYPE_double TargetVelocityOut_ID_44537;

GEN_TYPE_boolean noval_TargetVelocityOut_ID_44537;

void clear_inputs_TargetVelocity_ID_10210(){
	noval_MaxVelocity_ID_10230 = true;
	noval_TargetVelocityIn_ID_16462 = true;
	noval_DA_States_ID_53839 = true;
}

void clear_outputs_TargetVelocity_ID_10210(){
	noval_TargetVelocityOut_ID_44537 = true;
}

void init_TargetVelocity_ID_10210(){
	noval_TargetVelocityOut_ID_44537 = true;
}

void perform_step_TargetVelocity_ID_10210(){
	clear_outputs_TargetVelocity_ID_10210();
	if (!(noval_DA_States_ID_53839 == true) && DA_States_ID_53839.acc == On()) {
		if (!(noval_MaxVelocity_ID_10230 == true) && MaxVelocity_ID_10230 <= TargetVelocityIn_ID_16462) {
			noval_TargetVelocityOut_ID_44537 = noval_MaxVelocity_ID_10230;
			if (!noval_MaxVelocity_ID_10230) {
				TargetVelocityOut_ID_44537 = MaxVelocity_ID_10230;
			}
			
		}
		else {
			noval_TargetVelocityOut_ID_44537 = noval_TargetVelocityIn_ID_16462;
			if (!noval_TargetVelocityIn_ID_16462) {
				TargetVelocityOut_ID_44537 = TargetVelocityIn_ID_16462;
			}
			
		}
	}
	else {
		noval_TargetVelocityOut_ID_44537 = false;
		TargetVelocityOut_ID_44537 = 0.0;
		
	}
	return;
	
}

/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:03 CEST 2019 */
#include "inc-gen/VelocityMem_ID_11087.h"

GEN_TYPE_double VelocityIn_ID_12245;

GEN_TYPE_boolean noval_VelocityIn_ID_12245;

GEN_TYPE_double LoopIn_ID_12220;

GEN_TYPE_boolean noval_LoopIn_ID_12220;

GEN_TYPE_double LoopOut_ID_12218;

GEN_TYPE_boolean noval_LoopOut_ID_12218;

GEN_TYPE_double VelocityOut_ID_11474;

GEN_TYPE_boolean noval_VelocityOut_ID_11474;

void clear_inputs_VelocityMem_ID_11087(){
	noval_VelocityIn_ID_12245 = true;
	noval_LoopIn_ID_12220 = true;
}

void clear_outputs_VelocityMem_ID_11087(){
	noval_LoopOut_ID_12218 = true;
	noval_VelocityOut_ID_11474 = true;
}

void init_VelocityMem_ID_11087(){
	noval_LoopOut_ID_12218 = true;
	noval_VelocityOut_ID_11474 = true;
}

void perform_step_VelocityMem_ID_11087(){
	clear_outputs_VelocityMem_ID_11087();
	if (!(noval_VelocityIn_ID_12245 == true)) {
		noval_VelocityOut_ID_11474 = noval_VelocityIn_ID_12245;
		if (!noval_VelocityIn_ID_12245) {
			VelocityOut_ID_11474 = VelocityIn_ID_12245;
		}
		
		noval_LoopOut_ID_12218 = noval_VelocityIn_ID_12245;
		if (!noval_VelocityIn_ID_12245) {
			LoopOut_ID_12218 = VelocityIn_ID_12245;
		}
		
	}
	else {
		noval_VelocityOut_ID_11474 = noval_LoopIn_ID_12220;
		if (!noval_LoopIn_ID_12220) {
			VelocityOut_ID_11474 = LoopIn_ID_12220;
		}
		
		noval_LoopOut_ID_12218 = noval_LoopIn_ID_12220;
		if (!noval_LoopIn_ID_12220) {
			LoopOut_ID_12218 = LoopIn_ID_12220;
		}
		
	}
	return;
	
}

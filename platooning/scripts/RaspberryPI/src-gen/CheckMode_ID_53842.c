/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/CheckMode_ID_53842.h"

GEN_TYPE_DriverAssistanceCommands DA_StatesIn_ID_53850;

GEN_TYPE_boolean noval_DA_StatesIn_ID_53850;

GEN_TYPE_boolean platooningOff_ID_53857;

GEN_TYPE_boolean noval_platooningOff_ID_53857;

GEN_TYPE_DriverAssistanceCommands DA_StatesOut_ID_53874;

GEN_TYPE_boolean noval_DA_StatesOut_ID_53874;

void clear_inputs_CheckMode_ID_53842(){
	noval_DA_StatesIn_ID_53850 = true;
	noval_platooningOff_ID_53857 = true;
}

void clear_outputs_CheckMode_ID_53842(){
	noval_DA_StatesOut_ID_53874 = true;
}

void init_CheckMode_ID_53842(){
	noval_DA_StatesOut_ID_53874 = true;
}

void perform_step_CheckMode_ID_53842(){
	clear_outputs_CheckMode_ID_53842();
	if (!(noval_DA_StatesIn_ID_53850 == true)) {
		noval_DA_StatesOut_ID_53874 = noval_DA_StatesIn_ID_53850;
		if (!noval_DA_StatesIn_ID_53850) {
			DA_StatesOut_ID_53874 = DA_StatesIn_ID_53850;
		}
		
	}
	else {
		noval_DA_StatesOut_ID_53874 = false;
		DA_StatesOut_ID_53874.acc = off();
		DA_StatesOut_ID_53874.laneKeep = off();
		DA_StatesOut_ID_53874.speedMinus = notPressed();
		DA_StatesOut_ID_53874.speedPlus = notPressed();
		DA_StatesOut_ID_53874.timePlus = notPressed();
		DA_StatesOut_ID_53874.timeMinus = notPressed();
		
		
	}
	if (!(noval_platooningOff_ID_53857 == true) && (noval_platooningOff_ID_53857 == false && platooningOff_ID_53857 == false)) {
		noval_DA_StatesOut_ID_53874 = false;
		DA_StatesOut_ID_53874.acc = On();
		DA_StatesOut_ID_53874.laneKeep = On();
		DA_StatesOut_ID_53874.speedMinus = notPressed();
		DA_StatesOut_ID_53874.speedPlus = notPressed();
		DA_StatesOut_ID_53874.timePlus = notPressed();
		DA_StatesOut_ID_53874.timeMinus = notPressed();
		
		
	}
	return;
	
}

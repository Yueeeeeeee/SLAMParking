/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/Aggregate_ID_8429.h"

GEN_TYPE_On_Off_States acc_ID_8461;

GEN_TYPE_boolean noval_acc_ID_8461;

GEN_TYPE_On_Off_States laneKeep_ID_8470;

GEN_TYPE_boolean noval_laneKeep_ID_8470;

GEN_TYPE_DriverAssistanceEvents DACommand_ID_44167;

GEN_TYPE_boolean noval_DACommand_ID_44167;

GEN_TYPE_DriverAssistanceCommands DA_States_ID_8606;

GEN_TYPE_boolean noval_DA_States_ID_8606;

void clear_inputs_Aggregate_ID_8429(){
	noval_acc_ID_8461 = true;
	noval_laneKeep_ID_8470 = true;
	noval_DACommand_ID_44167 = true;
}

void clear_outputs_Aggregate_ID_8429(){
	noval_DA_States_ID_8606 = true;
}

void init_Aggregate_ID_8429(){
	noval_DA_States_ID_8606 = true;
}

void perform_step_Aggregate_ID_8429(){
	clear_outputs_Aggregate_ID_8429();
	noval_DA_States_ID_8606 = false;
	DA_States_ID_8606.acc = acc_ID_8461;
	DA_States_ID_8606.laneKeep = laneKeep_ID_8470;
	DA_States_ID_8606.speedMinus = DACommand_ID_44167.speedMinus;
	DA_States_ID_8606.speedPlus = DACommand_ID_44167.speedPlus;
	DA_States_ID_8606.timePlus = DACommand_ID_44167.timePlus;
	DA_States_ID_8606.timeMinus = DACommand_ID_44167.timeMinus;
	
	
	return;
	
}

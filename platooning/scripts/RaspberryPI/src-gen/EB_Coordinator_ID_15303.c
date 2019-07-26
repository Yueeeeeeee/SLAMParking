/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/EB_Coordinator_ID_15303.h"

GEN_TYPE_DriveModeSettings DriveMode_ID_31968;

GEN_TYPE_boolean noval_DriveMode_ID_31968;

GEN_TYPE_boolean CriticalSituation_ID_15314;

GEN_TYPE_boolean noval_CriticalSituation_ID_15314;

GEN_TYPE_boolean FirstCritTick_Old_ID_32322;

GEN_TYPE_boolean noval_FirstCritTick_Old_ID_32322;

GEN_TYPE_boolean Activated_ID_15312;

GEN_TYPE_boolean noval_Activated_ID_15312;

GEN_TYPE_boolean FirstCritTick_ID_32329;

GEN_TYPE_boolean noval_FirstCritTick_ID_32329;

void clear_inputs_EB_Coordinator_ID_15303(){
	noval_DriveMode_ID_31968 = true;
	noval_CriticalSituation_ID_15314 = true;
	noval_FirstCritTick_Old_ID_32322 = true;
}

void clear_outputs_EB_Coordinator_ID_15303(){
	noval_Activated_ID_15312 = true;
	noval_FirstCritTick_ID_32329 = true;
}

void init_EB_Coordinator_ID_15303(){
	noval_Activated_ID_15312 = true;
	noval_FirstCritTick_ID_32329 = true;
}

void perform_step_EB_Coordinator_ID_15303(){
	clear_outputs_EB_Coordinator_ID_15303();
	if (!(noval_CriticalSituation_ID_15314 == true) && (noval_CriticalSituation_ID_15314 == false && CriticalSituation_ID_15314 == true)) {
		if (noval_FirstCritTick_Old_ID_32322 == true) {
			noval_Activated_ID_15312 = false;
			Activated_ID_15312 = false;
			
			noval_FirstCritTick_ID_32329 = false;
			FirstCritTick_ID_32329 = false;
			
		}
		else {
			if (noval_FirstCritTick_Old_ID_32322 == false && FirstCritTick_Old_ID_32322 == false) {
				noval_Activated_ID_15312 = false;
				Activated_ID_15312 = false;
				
				noval_FirstCritTick_ID_32329 = false;
				FirstCritTick_ID_32329 = true;
				
			}
			else {
				noval_Activated_ID_15312 = false;
				Activated_ID_15312 = true;
				
				noval_FirstCritTick_ID_32329 = false;
				FirstCritTick_ID_32329 = true;
				
			}
		}
	}
	else {
		noval_Activated_ID_15312 = false;
		Activated_ID_15312 = false;
		
		noval_FirstCritTick_ID_32329 = false;
		FirstCritTick_ID_32329 = false;
		
	}
	if (!(noval_DriveMode_ID_31968 == true) && (DriveMode_ID_31968.DriveMode == Parking() || DriveMode_ID_31968.DriveMode == Backward())) {
		noval_Activated_ID_15312 = false;
		Activated_ID_15312 = false;
		
		noval_FirstCritTick_ID_32329 = false;
		FirstCritTick_ID_32329 = false;
		
	}
	return;
	
}

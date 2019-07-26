/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:03 CEST 2019 */
#include "inc-gen/Lane_Warning_ID_15516.h"

GEN_TYPE_LDCameraData LDCameraData_ID_15525;

GEN_TYPE_boolean noval_LDCameraData_ID_15525;

GEN_TYPE_DriveModeSettings DriveMode_ID_15622;

GEN_TYPE_boolean noval_DriveMode_ID_15622;

GEN_TYPE_double SteeringWheel_ID_16864;

GEN_TYPE_boolean noval_SteeringWheel_ID_16864;

GEN_TYPE_boolean RumbleLW_ID_15530;

GEN_TYPE_boolean noval_RumbleLW_ID_15530;

void clear_inputs_Lane_Warning_ID_15516(){
	noval_LDCameraData_ID_15525 = true;
	noval_DriveMode_ID_15622 = true;
	noval_SteeringWheel_ID_16864 = true;
}

void clear_outputs_Lane_Warning_ID_15516(){
	noval_RumbleLW_ID_15530 = true;
}

void init_Lane_Warning_ID_15516(){
	noval_RumbleLW_ID_15530 = true;
}

void perform_step_Lane_Warning_ID_15516(){
	clear_outputs_Lane_Warning_ID_15516();
	if (noval_LDCameraData_ID_15525 == true || noval_DriveMode_ID_15622 == true) {
		noval_RumbleLW_ID_15530 = false;
		RumbleLW_ID_15530 = false;
		
		return;
	}
	if (DriveMode_ID_15622.DriveMode == Forward()) {
		if (LDCameraData_ID_15525.present_left && LDCameraData_ID_15525.distance_left - ROVER_WIDTH() / 2.0 < LW_Distance_Min()) {
			noval_RumbleLW_ID_15530 = false;
			RumbleLW_ID_15530 = true;
			
			return;
		}
		if (LDCameraData_ID_15525.present_right && LDCameraData_ID_15525.distance_right - ROVER_WIDTH() / 2.0 < LW_Distance_Min()) {
			noval_RumbleLW_ID_15530 = false;
			RumbleLW_ID_15530 = true;
			
			return;
		}
	}
	noval_RumbleLW_ID_15530 = false;
	RumbleLW_ID_15530 = false;
	
	return;
	
}

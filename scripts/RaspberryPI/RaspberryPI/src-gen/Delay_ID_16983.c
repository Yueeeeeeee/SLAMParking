/* generated by AutoFOCUS 3 (2.14.0) on Fri Jul 05 17:37:26 CEST 2019 */
#include "inc-gen/Delay_ID_16983.h"

GEN_TYPE_boolean RumbleEB_In_ID_17264;

GEN_TYPE_boolean noval_RumbleEB_In_ID_17264;

GEN_TYPE_DriveModeSettings DriveMode_ID_31975;

GEN_TYPE_boolean noval_DriveMode_ID_31975;

GEN_TYPE_boolean FirstCritTick_ID_32331;

GEN_TYPE_boolean noval_FirstCritTick_ID_32331;

GEN_TYPE_boolean RumbleEB_ID_16997;

GEN_TYPE_boolean noval_RumbleEB_ID_16997;

GEN_TYPE_DriveModeSettings DriveMode_out_ID_31966;

GEN_TYPE_boolean noval_DriveMode_out_ID_31966;

GEN_TYPE_boolean FirstCritTick_Old_ID_32320;

GEN_TYPE_boolean noval_FirstCritTick_Old_ID_32320;

void clear_inputs_Delay_ID_16983(){
	noval_RumbleEB_In_ID_17264 = true;
	noval_DriveMode_ID_31975 = true;
	noval_FirstCritTick_ID_32331 = true;
}

void clear_outputs_Delay_ID_16983(){
	noval_RumbleEB_ID_16997 = true;
	noval_DriveMode_out_ID_31966 = true;
	noval_FirstCritTick_Old_ID_32320 = true;
}

void init_Delay_ID_16983(){
	noval_RumbleEB_ID_16997 = true;
	noval_DriveMode_out_ID_31966 = true;
	noval_FirstCritTick_Old_ID_32320 = true;
}

void perform_step_Delay_ID_16983(){
	clear_outputs_Delay_ID_16983();
	noval_RumbleEB_ID_16997 = noval_RumbleEB_In_ID_17264;
	if (!noval_RumbleEB_In_ID_17264) {
		RumbleEB_ID_16997 = RumbleEB_In_ID_17264;
	}
	
	noval_DriveMode_out_ID_31966 = noval_DriveMode_ID_31975;
	if (!noval_DriveMode_ID_31975) {
		DriveMode_out_ID_31966 = DriveMode_ID_31975;
	}
	
	noval_FirstCritTick_Old_ID_32320 = noval_FirstCritTick_ID_32331;
	if (!noval_FirstCritTick_ID_32331) {
		FirstCritTick_Old_ID_32320 = FirstCritTick_ID_32331;
	}
	
	return;
	
}

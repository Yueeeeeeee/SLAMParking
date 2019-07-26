/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:07 CEST 2019 */
#include "inc-gen/DrivingMode_ID_5723.h"

GEN_TYPE_DriveModeSettings DriveMode_ID_12240;

GEN_TYPE_boolean noval_DriveMode_ID_12240;

GEN_TYPE_double GasPedalIn_ID_12230;

GEN_TYPE_boolean noval_GasPedalIn_ID_12230;

GEN_TYPE_int GearShiftIn_ID_12225;

GEN_TYPE_boolean noval_GearShiftIn_ID_12225;

GEN_TYPE_DriveMode StateOld_ID_12596;

GEN_TYPE_boolean noval_StateOld_ID_12596;

GEN_TYPE_double GasPedalOut_ID_11448;

GEN_TYPE_boolean noval_GasPedalOut_ID_11448;

GEN_TYPE_int GearShiftOut_ID_11453;

GEN_TYPE_boolean noval_GearShiftOut_ID_11453;

GEN_TYPE_DriveMode StateNew_ID_12585;

GEN_TYPE_boolean noval_StateNew_ID_12585;

void clear_inputs_DrivingMode_ID_5723(){
	noval_DriveMode_ID_12240 = true;
	noval_GasPedalIn_ID_12230 = true;
	noval_GearShiftIn_ID_12225 = true;
	noval_StateOld_ID_12596 = true;
}

void clear_outputs_DrivingMode_ID_5723(){
	noval_GasPedalOut_ID_11448 = true;
	noval_GearShiftOut_ID_11453 = true;
	noval_StateNew_ID_12585 = true;
}

void init_DrivingMode_ID_5723(){
	noval_GasPedalOut_ID_11448 = true;
	noval_GearShiftOut_ID_11453 = true;
	noval_StateNew_ID_12585 = true;
}

void perform_step_DrivingMode_ID_5723(){
	clear_outputs_DrivingMode_ID_5723();
	if (noval_DriveMode_ID_12240 == true || DriveMode_ID_12240.DriveMode == Parking()) {
		noval_GasPedalOut_ID_11448 = false;
		GasPedalOut_ID_11448 = 0;
		
		noval_GearShiftOut_ID_11453 = true;
	}
	if (DriveMode_ID_12240.DriveMode == Forward()) {
		noval_GasPedalOut_ID_11448 = noval_GasPedalIn_ID_12230;
		if (!noval_GasPedalIn_ID_12230) {
			GasPedalOut_ID_11448 = GasPedalIn_ID_12230;
		}
		
		noval_GearShiftOut_ID_11453 = noval_GearShiftIn_ID_12225;
		if (!noval_GearShiftIn_ID_12225) {
			GearShiftOut_ID_11453 = GearShiftIn_ID_12225;
		}
		
	}
	if (DriveMode_ID_12240.DriveMode == Backward()) {
		noval_GasPedalOut_ID_11448 = false;
		GasPedalOut_ID_11448 = -GasPedalIn_ID_12230;
		
		noval_GearShiftOut_ID_11453 = true;
	}
	if (!(noval_DriveMode_ID_12240 == true)) {
		noval_StateNew_ID_12585 = false;
		StateNew_ID_12585 = DriveMode_ID_12240.DriveMode;
		
	}
	else {
		noval_StateNew_ID_12585 = true;
	}
	if (!(noval_StateOld_ID_12596 == false && StateOld_ID_12596 == DriveMode_ID_12240.DriveMode)) {
		noval_GearShiftOut_ID_11453 = false;
		GearShiftOut_ID_11453 = 0;
		
		noval_GasPedalOut_ID_11448 = false;
		GasPedalOut_ID_11448 = 0;
		
	}
	return;
	
}

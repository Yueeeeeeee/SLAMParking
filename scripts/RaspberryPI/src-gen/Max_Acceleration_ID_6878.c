/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:02 CEST 2019 */
#include "inc-gen/Max_Acceleration_ID_6878.h"

GEN_TYPE_double MaxAccelerationEB_ID_29308;

GEN_TYPE_boolean noval_MaxAccelerationEB_ID_29308;

GEN_TYPE_double MaxAccelerationMD_ID_16254;

GEN_TYPE_boolean noval_MaxAccelerationMD_ID_16254;

GEN_TYPE_double MaxAccelerationDA_ID_16200;

GEN_TYPE_boolean noval_MaxAccelerationDA_ID_16200;

GEN_TYPE_boolean ACC_On_Correct_ID_16269;

GEN_TYPE_boolean noval_ACC_On_Correct_ID_16269;

GEN_TYPE_boolean ForceEBOff_ID_29249;

GEN_TYPE_boolean noval_ForceEBOff_ID_29249;

GEN_TYPE_double MaxAccelerationAD_ID_20259;

GEN_TYPE_boolean noval_MaxAccelerationAD_ID_20259;

GEN_TYPE_boolean Platooning_IsLeader_ID_29700;

GEN_TYPE_boolean noval_Platooning_IsLeader_ID_29700;

GEN_TYPE_boolean platooningOff_ID_29707;

GEN_TYPE_boolean noval_platooningOff_ID_29707;

GEN_TYPE_double MaxAcceleration_ID_18004;

GEN_TYPE_boolean noval_MaxAcceleration_ID_18004;

void clear_inputs_Max_Acceleration_ID_6878(){
	noval_MaxAccelerationEB_ID_29308 = true;
	noval_MaxAccelerationMD_ID_16254 = true;
	noval_MaxAccelerationDA_ID_16200 = true;
	noval_ACC_On_Correct_ID_16269 = true;
	noval_ForceEBOff_ID_29249 = true;
	noval_MaxAccelerationAD_ID_20259 = true;
	noval_Platooning_IsLeader_ID_29700 = true;
	noval_platooningOff_ID_29707 = true;
}

void clear_outputs_Max_Acceleration_ID_6878(){
	noval_MaxAcceleration_ID_18004 = true;
}

void init_Max_Acceleration_ID_6878(){
	noval_MaxAcceleration_ID_18004 = true;
}

void perform_step_Max_Acceleration_ID_6878(){
	clear_outputs_Max_Acceleration_ID_6878();
	if (!(noval_MaxAccelerationEB_ID_29308 == true) && !ForceEBOff_ID_29249) {
		noval_MaxAcceleration_ID_18004 = noval_MaxAccelerationEB_ID_29308;
		if (!noval_MaxAccelerationEB_ID_29308) {
			MaxAcceleration_ID_18004 = MaxAccelerationEB_ID_29308;
		}
		
	}
	else {
		if ((noval_platooningOff_ID_29707 == true || platooningOff_ID_29707) || Platooning_IsLeader_ID_29700) {
			if (!(noval_ACC_On_Correct_ID_16269 == true) && ACC_On_Correct_ID_16269) {
				if (!(noval_MaxAccelerationDA_ID_16200 == true)) {
					noval_MaxAcceleration_ID_18004 = noval_MaxAccelerationDA_ID_16200;
					if (!noval_MaxAccelerationDA_ID_16200) {
						MaxAcceleration_ID_18004 = MaxAccelerationDA_ID_16200;
					}
					
				}
				else {
					noval_MaxAcceleration_ID_18004 = false;
					MaxAcceleration_ID_18004 = 0.0;
					
				}
			}
			else {
				if (!(noval_MaxAccelerationMD_ID_16254 == true)) {
					noval_MaxAcceleration_ID_18004 = noval_MaxAccelerationMD_ID_16254;
					if (!noval_MaxAccelerationMD_ID_16254) {
						MaxAcceleration_ID_18004 = MaxAccelerationMD_ID_16254;
					}
					
				}
				else {
					noval_MaxAcceleration_ID_18004 = false;
					MaxAcceleration_ID_18004 = 0.0;
					
				}
			}
		}
		else {
			if (!(noval_MaxAccelerationDA_ID_16200 == true)) {
				noval_MaxAcceleration_ID_18004 = noval_MaxAccelerationDA_ID_16200;
				if (!noval_MaxAccelerationDA_ID_16200) {
					MaxAcceleration_ID_18004 = MaxAccelerationDA_ID_16200;
				}
				
			}
			else {
				noval_MaxAcceleration_ID_18004 = false;
				MaxAcceleration_ID_18004 = 0.0;
				
			}
		}
	}
	return;
	
}

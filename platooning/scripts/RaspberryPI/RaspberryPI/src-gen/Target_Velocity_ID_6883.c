/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:08 CEST 2019 */
#include "inc-gen/Target_Velocity_ID_6883.h"

GEN_TYPE_double TargetVelocityMD_ID_16274;

GEN_TYPE_boolean noval_TargetVelocityMD_ID_16274;

GEN_TYPE_double TargetVelocityDA_ID_16279;

GEN_TYPE_boolean noval_TargetVelocityDA_ID_16279;

GEN_TYPE_boolean ACC_On_Correct_ID_16264;

GEN_TYPE_boolean noval_ACC_On_Correct_ID_16264;

GEN_TYPE_double MaxAccelerationEB_ID_29298;

GEN_TYPE_boolean noval_MaxAccelerationEB_ID_29298;

GEN_TYPE_boolean ForceEBOff_ID_29244;

GEN_TYPE_boolean noval_ForceEBOff_ID_29244;

GEN_TYPE_double TargetVelocityAD_ID_20266;

GEN_TYPE_boolean noval_TargetVelocityAD_ID_20266;

GEN_TYPE_boolean Platooning_IsLeader_ID_29679;

GEN_TYPE_boolean noval_Platooning_IsLeader_ID_29679;

GEN_TYPE_boolean platooningOff_ID_29686;

GEN_TYPE_boolean noval_platooningOff_ID_29686;

GEN_TYPE_double TargetVelocity_ID_6902;

GEN_TYPE_boolean noval_TargetVelocity_ID_6902;

void clear_inputs_Target_Velocity_ID_6883(){
	noval_TargetVelocityMD_ID_16274 = true;
	noval_TargetVelocityDA_ID_16279 = true;
	noval_ACC_On_Correct_ID_16264 = true;
	noval_MaxAccelerationEB_ID_29298 = true;
	noval_ForceEBOff_ID_29244 = true;
	noval_TargetVelocityAD_ID_20266 = true;
	noval_Platooning_IsLeader_ID_29679 = true;
	noval_platooningOff_ID_29686 = true;
}

void clear_outputs_Target_Velocity_ID_6883(){
	noval_TargetVelocity_ID_6902 = true;
}

void init_Target_Velocity_ID_6883(){
	noval_TargetVelocity_ID_6902 = true;
}

void perform_step_Target_Velocity_ID_6883(){
	clear_outputs_Target_Velocity_ID_6883();
	if (!(noval_MaxAccelerationEB_ID_29298 == true) && !ForceEBOff_ID_29244) {
		noval_TargetVelocity_ID_6902 = false;
		TargetVelocity_ID_6902 = EB_VELOCITY();
		
	}
	else {
		if ((noval_platooningOff_ID_29686 == true || platooningOff_ID_29686) || Platooning_IsLeader_ID_29679) {
			if (!(noval_ACC_On_Correct_ID_16264 == true) && ACC_On_Correct_ID_16264) {
				if (!(noval_TargetVelocityDA_ID_16279 == true)) {
					noval_TargetVelocity_ID_6902 = noval_TargetVelocityDA_ID_16279;
					if (!noval_TargetVelocityDA_ID_16279) {
						TargetVelocity_ID_6902 = TargetVelocityDA_ID_16279;
					}
					
				}
				else {
					noval_TargetVelocity_ID_6902 = false;
					TargetVelocity_ID_6902 = EB_VELOCITY();
					
				}
			}
			else {
				if (!(noval_TargetVelocityMD_ID_16274 == true)) {
					noval_TargetVelocity_ID_6902 = noval_TargetVelocityMD_ID_16274;
					if (!noval_TargetVelocityMD_ID_16274) {
						TargetVelocity_ID_6902 = TargetVelocityMD_ID_16274;
					}
					
				}
				else {
					noval_TargetVelocity_ID_6902 = false;
					TargetVelocity_ID_6902 = EB_VELOCITY();
					
				}
			}
		}
		else {
			if (!(noval_TargetVelocityAD_ID_20266 == true)) {
				noval_TargetVelocity_ID_6902 = noval_TargetVelocityAD_ID_20266;
				if (!noval_TargetVelocityAD_ID_20266) {
					TargetVelocity_ID_6902 = TargetVelocityAD_ID_20266;
				}
				
			}
			else {
				noval_TargetVelocity_ID_6902 = false;
				TargetVelocity_ID_6902 = EB_VELOCITY();
				
			}
		}
	}
	return;
	
}

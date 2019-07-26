/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:07 CEST 2019 */
#include "inc-gen/Error_ID_16055.h"

GEN_TYPE_LDCameraData LDCameraData_ID_16064;

GEN_TYPE_boolean noval_LDCameraData_ID_16064;

GEN_TYPE_double LastErrorIn_ID_16372;

GEN_TYPE_boolean noval_LastErrorIn_ID_16372;

GEN_TYPE_double StreetWidthIn_ID_30737;

GEN_TYPE_boolean noval_StreetWidthIn_ID_30737;

GEN_TYPE_double Error_ID_16085;

GEN_TYPE_boolean noval_Error_ID_16085;

GEN_TYPE_double StreetWidthOut_ID_30735;

GEN_TYPE_boolean noval_StreetWidthOut_ID_30735;

void clear_inputs_Error_ID_16055(){
	noval_LDCameraData_ID_16064 = true;
	noval_LastErrorIn_ID_16372 = true;
	noval_StreetWidthIn_ID_30737 = true;
}

void clear_outputs_Error_ID_16055(){
	noval_Error_ID_16085 = true;
	noval_StreetWidthOut_ID_30735 = true;
}

void init_Error_ID_16055(){
	noval_Error_ID_16085 = true;
	noval_StreetWidthOut_ID_30735 = true;
}

void perform_step_Error_ID_16055(){
	clear_outputs_Error_ID_16055();
	if (noval_LDCameraData_ID_16064 == true) {
		noval_Error_ID_16085 = true;
		noval_StreetWidthOut_ID_30735 = noval_StreetWidthIn_ID_30737;
		if (!noval_StreetWidthIn_ID_30737) {
			StreetWidthOut_ID_30735 = StreetWidthIn_ID_30737;
		}
		
		return;
	}
	if (LDCameraData_ID_16064.present_right && LDCameraData_ID_16064.present_left) {
		noval_StreetWidthOut_ID_30735 = false;
		StreetWidthOut_ID_30735 = LDCameraData_ID_16064.distance_left + LDCameraData_ID_16064.distance_right;
		
		if (LDCameraData_ID_16064.distance_right - LDCameraData_ID_16064.distance_left > 0.0 && LastErrorIn_ID_16372 < 0.0 || LDCameraData_ID_16064.distance_right - LDCameraData_ID_16064.distance_left < 0.0 && LastErrorIn_ID_16372 > 0.0) {
			noval_Error_ID_16085 = false;
			Error_ID_16085 = 0.0;
			
		}
		else {
			if (myabs((LDCameraData_ID_16064.distance_right - LDCameraData_ID_16064.distance_left)) <= LD_DISTANCE_ERROR_DEADZONE()) {
				noval_Error_ID_16085 = false;
				Error_ID_16085 = 0.0;
				
			}
			else {
				noval_Error_ID_16085 = false;
				Error_ID_16085 = (LDCameraData_ID_16064.distance_right - LDCameraData_ID_16064.distance_left) / 2.0;
				
			}
		}
	}
	else {
		noval_StreetWidthOut_ID_30735 = noval_StreetWidthIn_ID_30737;
		if (!noval_StreetWidthIn_ID_30737) {
			StreetWidthOut_ID_30735 = StreetWidthIn_ID_30737;
		}
		
		if (!LDCameraData_ID_16064.present_right && LDCameraData_ID_16064.present_left || LDCameraData_ID_16064.present_right && !LDCameraData_ID_16064.present_left) {
			if (!LDCameraData_ID_16064.present_right && LDCameraData_ID_16064.present_left) {
				if (!(noval_StreetWidthIn_ID_30737 == true)) {
					noval_Error_ID_16085 = false;
					Error_ID_16085 = StreetWidthIn_ID_30737 / 2 - LDCameraData_ID_16064.distance_left;
					
				}
				else {
					noval_Error_ID_16085 = false;
					Error_ID_16085 = ASSUMED_STREET_WIDTH() - LDCameraData_ID_16064.distance_left;
					
				}
			}
			else {
				if (!(noval_StreetWidthIn_ID_30737 == true)) {
					noval_Error_ID_16085 = false;
					Error_ID_16085 = LDCameraData_ID_16064.distance_right - StreetWidthIn_ID_30737;
					
				}
				else {
					noval_Error_ID_16085 = false;
					Error_ID_16085 = LDCameraData_ID_16064.distance_right - ASSUMED_STREET_WIDTH();
					
				}
			}
		}
		else {
			noval_Error_ID_16085 = false;
			Error_ID_16085 = 0.0;
			
		}
	}
	return;
	
}

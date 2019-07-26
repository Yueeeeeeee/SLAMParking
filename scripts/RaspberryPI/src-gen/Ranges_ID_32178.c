/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:02 CEST 2019 */
#include "inc-gen/Ranges_ID_32178.h"

GEN_TYPE_double DistanceToFrontLaser_Out_ID_32219;

GEN_TYPE_boolean noval_DistanceToFrontLaser_Out_ID_32219;

GEN_TYPE_double DistanceToFrontUS_Out1_ID_32226;

GEN_TYPE_boolean noval_DistanceToFrontUS_Out1_ID_32226;

GEN_TYPE_double DistanceToFrontUS_Out2_ID_32233;

GEN_TYPE_boolean noval_DistanceToFrontUS_Out2_ID_32233;

GEN_TYPE_double DistanceToFrontLaser_In_ID_44740;

GEN_TYPE_boolean noval_DistanceToFrontLaser_In_ID_44740;

GEN_TYPE_double DistanceToFrontUS_1_ID_44335;

GEN_TYPE_boolean noval_DistanceToFrontUS_1_ID_44335;

GEN_TYPE_double DistanceToFrontUS_2_ID_44342;

GEN_TYPE_boolean noval_DistanceToFrontUS_2_ID_44342;

void clear_inputs_Ranges_ID_32178(){
	noval_DistanceToFrontLaser_Out_ID_32219 = true;
	noval_DistanceToFrontUS_Out1_ID_32226 = true;
	noval_DistanceToFrontUS_Out2_ID_32233 = true;
}

void clear_outputs_Ranges_ID_32178(){
	noval_DistanceToFrontLaser_In_ID_44740 = true;
	noval_DistanceToFrontUS_1_ID_44335 = true;
	noval_DistanceToFrontUS_2_ID_44342 = true;
}

void init_Ranges_ID_32178(){
	noval_DistanceToFrontLaser_In_ID_44740 = true;
	noval_DistanceToFrontUS_1_ID_44335 = true;
	noval_DistanceToFrontUS_2_ID_44342 = true;
}

void perform_step_Ranges_ID_32178(){
	clear_outputs_Ranges_ID_32178();
	if (!(noval_DistanceToFrontLaser_Out_ID_32219 == true)) {
		if (DistanceToFrontLaser_Out_ID_32219 <= LASER_MAX_INVALID() || DistanceToFrontLaser_Out_ID_32219 >= LASER_MAX()) {
			noval_DistanceToFrontLaser_In_ID_44740 = false;
			DistanceToFrontLaser_In_ID_44740 = LASER_MAX();
			
		}
		else {
			if (DistanceToFrontLaser_Out_ID_32219 >= LASER_MIN()) {
				noval_DistanceToFrontLaser_In_ID_44740 = noval_DistanceToFrontLaser_Out_ID_32219;
				if (!noval_DistanceToFrontLaser_Out_ID_32219) {
					DistanceToFrontLaser_In_ID_44740 = DistanceToFrontLaser_Out_ID_32219;
				}
				
			}
			else {
				if (DistanceToFrontLaser_Out_ID_32219 >= LASER_ZERO_MIN_VALUE()) {
					noval_DistanceToFrontLaser_In_ID_44740 = false;
					DistanceToFrontLaser_In_ID_44740 = 0.0;
					
				}
				else {
					noval_DistanceToFrontLaser_In_ID_44740 = true;
				}
			}
		}
	}
	else {
		noval_DistanceToFrontLaser_In_ID_44740 = true;
	}
	if ((!(noval_DistanceToFrontUS_Out1_ID_32226 == true) && DistanceToFrontUS_Out1_ID_32226 >= US_MIN()) && DistanceToFrontUS_Out1_ID_32226 <= US_MAX()) {
		noval_DistanceToFrontUS_1_ID_44335 = noval_DistanceToFrontUS_Out1_ID_32226;
		if (!noval_DistanceToFrontUS_Out1_ID_32226) {
			DistanceToFrontUS_1_ID_44335 = DistanceToFrontUS_Out1_ID_32226;
		}
		
	}
	else {
		noval_DistanceToFrontUS_1_ID_44335 = true;
	}
	if ((!(noval_DistanceToFrontUS_Out2_ID_32233 == true) && DistanceToFrontUS_Out2_ID_32233 >= US_MIN()) && DistanceToFrontUS_Out2_ID_32233 <= US_MAX()) {
		noval_DistanceToFrontUS_2_ID_44342 = noval_DistanceToFrontUS_Out2_ID_32233;
		if (!noval_DistanceToFrontUS_Out2_ID_32233) {
			DistanceToFrontUS_2_ID_44342 = DistanceToFrontUS_Out2_ID_32233;
		}
		
	}
	else {
		noval_DistanceToFrontUS_2_ID_44342 = true;
	}
	return;
	
}

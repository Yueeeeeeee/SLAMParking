/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/Delay_ID_31910.h"

GEN_TYPE_double MaxAccelerationAD_ID_31919;

GEN_TYPE_boolean noval_MaxAccelerationAD_ID_31919;

GEN_TYPE_boolean Platooning_EB_Triggered_ID_31926;

GEN_TYPE_boolean noval_Platooning_EB_Triggered_ID_31926;

GEN_TYPE_boolean platooningOff_ID_31933;

GEN_TYPE_boolean noval_platooningOff_ID_31933;

GEN_TYPE_boolean platooningOff_out_ID_31940;

GEN_TYPE_boolean noval_platooningOff_out_ID_31940;

GEN_TYPE_boolean Platooning_EB_Triggered_out_ID_31949;

GEN_TYPE_boolean noval_Platooning_EB_Triggered_out_ID_31949;

GEN_TYPE_double MaxAccelerationAD_out_ID_31958;

GEN_TYPE_boolean noval_MaxAccelerationAD_out_ID_31958;

void clear_inputs_Delay_ID_31910(){
	noval_MaxAccelerationAD_ID_31919 = true;
	noval_Platooning_EB_Triggered_ID_31926 = true;
	noval_platooningOff_ID_31933 = true;
}

void clear_outputs_Delay_ID_31910(){
	noval_platooningOff_out_ID_31940 = true;
	noval_Platooning_EB_Triggered_out_ID_31949 = true;
	noval_MaxAccelerationAD_out_ID_31958 = true;
}

void init_Delay_ID_31910(){
	noval_platooningOff_out_ID_31940 = true;
	noval_Platooning_EB_Triggered_out_ID_31949 = true;
	noval_MaxAccelerationAD_out_ID_31958 = true;
}

void perform_step_Delay_ID_31910(){
	clear_outputs_Delay_ID_31910();
	noval_MaxAccelerationAD_out_ID_31958 = noval_MaxAccelerationAD_ID_31919;
	if (!noval_MaxAccelerationAD_ID_31919) {
		MaxAccelerationAD_out_ID_31958 = MaxAccelerationAD_ID_31919;
	}
	
	noval_platooningOff_out_ID_31940 = noval_platooningOff_ID_31933;
	if (!noval_platooningOff_ID_31933) {
		platooningOff_out_ID_31940 = platooningOff_ID_31933;
	}
	
	noval_Platooning_EB_Triggered_out_ID_31949 = noval_Platooning_EB_Triggered_ID_31926;
	if (!noval_Platooning_EB_Triggered_ID_31926) {
		Platooning_EB_Triggered_out_ID_31949 = Platooning_EB_Triggered_ID_31926;
	}
	
	return;
	
}

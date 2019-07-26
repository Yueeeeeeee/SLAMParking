/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/Control_States_ID_45920.h"

GEN_TYPE_double TargetVelocity_ID_47833;

GEN_TYPE_boolean noval_TargetVelocity_ID_47833;

GEN_TYPE_double MaxAcceleration_ID_47842;

GEN_TYPE_boolean noval_MaxAcceleration_ID_47842;

GEN_TYPE_double SteeringAngle_ID_47851;

GEN_TYPE_boolean noval_SteeringAngle_ID_47851;

GEN_TYPE_PlatoonState platoonState_ID_50203;

GEN_TYPE_boolean noval_platoonState_ID_50203;

GEN_TYPE_double TargetVelocityAD_ID_47801;

GEN_TYPE_boolean noval_TargetVelocityAD_ID_47801;

GEN_TYPE_double MaxAccelerationAD_ID_47808;

GEN_TYPE_boolean noval_MaxAccelerationAD_ID_47808;

GEN_TYPE_double SteeringAngleAD_ID_47815;

GEN_TYPE_boolean noval_SteeringAngleAD_ID_47815;

GEN_TYPE_boolean platooningOff_ID_50155;

GEN_TYPE_boolean noval_platooningOff_ID_50155;

static GEN_TYPE_int current_state;

static GEN_TYPE_boolean fire_transition_turnOff_ID_46949();

static GEN_TYPE_boolean fire_transition_turnOn_ID_46958();

static GEN_TYPE_boolean fire_transition_accON_ID_47171();

static GEN_TYPE_boolean fire_transition_accOFF_ID_47179();

static GEN_TYPE_boolean fire_transition_turnOff_ID_48870();

static GEN_TYPE_boolean fire_state_ControlOff_ID_45980();

static GEN_TYPE_boolean fire_state_Manual_ID_45983();

static GEN_TYPE_boolean fire_state_ACC_ID_46394();

void clear_inputs_Control_States_ID_45920(){
	noval_TargetVelocity_ID_47833 = true;
	noval_MaxAcceleration_ID_47842 = true;
	noval_SteeringAngle_ID_47851 = true;
	noval_platoonState_ID_50203 = true;
}

void clear_outputs_Control_States_ID_45920(){
	noval_TargetVelocityAD_ID_47801 = true;
	noval_MaxAccelerationAD_ID_47808 = true;
	noval_SteeringAngleAD_ID_47815 = true;
	noval_platooningOff_ID_50155 = true;
}

void init_Control_States_ID_45920(){
	noval_TargetVelocityAD_ID_47801 = true;
	noval_MaxAccelerationAD_ID_47808 = true;
	noval_SteeringAngleAD_ID_47815 = true;
	noval_platooningOff_ID_50155 = true;
	current_state = 45980;
}

void perform_step_Control_States_ID_45920(){
	clear_outputs_Control_States_ID_45920();
	if (current_state == 46394) {
		fire_state_ACC_ID_46394();
	}
	else {
		if (current_state == 45983) {
			fire_state_Manual_ID_45983();
		}
		else {
			if (current_state == 45980) {
				fire_state_ControlOff_ID_45980();
			}
		}
	}
}

static GEN_TYPE_boolean fire_transition_turnOff_ID_46949(){
	if (!(noval_platoonState_ID_50203 == true) && (noval_platoonState_ID_50203 == false && platoonState_ID_50203 == off())) {
		current_state = 45980;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_turnOn_ID_46958(){
	if (!(noval_platoonState_ID_50203 == true) && (noval_platoonState_ID_50203 == false && platoonState_ID_50203 == leader() || noval_platoonState_ID_50203 == false && platoonState_ID_50203 == fuseStarter())) {
		current_state = 45983;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_accON_ID_47171(){
	if (!(noval_platoonState_ID_50203 == true) && (noval_platoonState_ID_50203 == false && platoonState_ID_50203 == follower())) {
		current_state = 46394;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_accOFF_ID_47179(){
	if (!(noval_platoonState_ID_50203 == true) && ((noval_platoonState_ID_50203 == false && platoonState_ID_50203 == leader() || noval_platoonState_ID_50203 == false && platoonState_ID_50203 == splitting()) || noval_platoonState_ID_50203 == false && platoonState_ID_50203 == leaving())) {
		current_state = 45983;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_transition_turnOff_ID_48870(){
	if (!(noval_platoonState_ID_50203 == true) && (noval_platoonState_ID_50203 == false && platoonState_ID_50203 == off())) {
		current_state = 45980;
		return true;
	}
	else {
		return false;
	}
}

static GEN_TYPE_boolean fire_state_ControlOff_ID_45980(){
	return fire_transition_turnOn_ID_46958();
}

static GEN_TYPE_boolean fire_state_Manual_ID_45983(){
	if (!(fire_transition_turnOff_ID_46949() || fire_transition_accON_ID_47171())) {
		noval_MaxAccelerationAD_ID_47808 = noval_MaxAcceleration_ID_47842;
		if (!noval_MaxAcceleration_ID_47842) {
			MaxAccelerationAD_ID_47808 = MaxAcceleration_ID_47842;
		}
		
		noval_SteeringAngleAD_ID_47815 = noval_SteeringAngle_ID_47851;
		if (!noval_SteeringAngle_ID_47851) {
			SteeringAngleAD_ID_47815 = SteeringAngle_ID_47851;
		}
		
		noval_TargetVelocityAD_ID_47801 = noval_TargetVelocity_ID_47833;
		if (!noval_TargetVelocity_ID_47833) {
			TargetVelocityAD_ID_47801 = TargetVelocity_ID_47833;
		}
		
	}
	return true;
}

static GEN_TYPE_boolean fire_state_ACC_ID_46394(){
	if (!(fire_transition_accOFF_ID_47179() || fire_transition_turnOff_ID_48870())) {
		noval_platooningOff_ID_50155 = false;
		platooningOff_ID_50155 = false;
		
	}
	return true;
}

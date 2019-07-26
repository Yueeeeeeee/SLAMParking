/* generated by AutoFOCUS 3 (2.14.0) on Mon Jul 08 13:45:02 CEST 2019 */
#include "inc-gen/Transform_Input_ID_12045.h"

GEN_TYPE_double LeftStick_XIn_ID_11995;

GEN_TYPE_boolean noval_LeftStick_XIn_ID_11995;

GEN_TYPE_double RightStick_XIn_ID_12000;

GEN_TYPE_boolean noval_RightStick_XIn_ID_12000;

GEN_TYPE_double RightStick_YIn_ID_12005;

GEN_TYPE_boolean noval_RightStick_YIn_ID_12005;

GEN_TYPE_double LeftStick_YIn_ID_14666;

GEN_TYPE_boolean noval_LeftStick_YIn_ID_14666;

GEN_TYPE_double ButtonL2In_ID_14976;

GEN_TYPE_boolean noval_ButtonL2In_ID_14976;

GEN_TYPE_double ButtonR2In_ID_14983;

GEN_TYPE_boolean noval_ButtonR2In_ID_14983;

GEN_TYPE_double LeftStick_X_ID_11978;

GEN_TYPE_boolean noval_LeftStick_X_ID_11978;

GEN_TYPE_double RightStick_X_ID_11983;

GEN_TYPE_boolean noval_RightStick_X_ID_11983;

GEN_TYPE_double RightStick_Y_ID_11988;

GEN_TYPE_boolean noval_RightStick_Y_ID_11988;

GEN_TYPE_double LeftStick_Y_ID_14673;

GEN_TYPE_boolean noval_LeftStick_Y_ID_14673;

GEN_TYPE_double ButtonL2_ID_14960;

GEN_TYPE_boolean noval_ButtonL2_ID_14960;

GEN_TYPE_double ButtonR2_ID_14967;

GEN_TYPE_boolean noval_ButtonR2_ID_14967;

void clear_inputs_Transform_Input_ID_12045(){
	noval_LeftStick_XIn_ID_11995 = true;
	noval_RightStick_XIn_ID_12000 = true;
	noval_RightStick_YIn_ID_12005 = true;
	noval_LeftStick_YIn_ID_14666 = true;
	noval_ButtonL2In_ID_14976 = true;
	noval_ButtonR2In_ID_14983 = true;
}

void clear_outputs_Transform_Input_ID_12045(){
	noval_LeftStick_X_ID_11978 = true;
	noval_RightStick_X_ID_11983 = true;
	noval_RightStick_Y_ID_11988 = true;
	noval_LeftStick_Y_ID_14673 = true;
	noval_ButtonL2_ID_14960 = true;
	noval_ButtonR2_ID_14967 = true;
}

void init_Transform_Input_ID_12045(){
	noval_LeftStick_X_ID_11978 = true;
	noval_RightStick_X_ID_11983 = true;
	noval_RightStick_Y_ID_11988 = true;
	noval_LeftStick_Y_ID_14673 = true;
	noval_ButtonL2_ID_14960 = true;
	noval_ButtonR2_ID_14967 = true;
}

void perform_step_Transform_Input_ID_12045(){
	clear_outputs_Transform_Input_ID_12045();
	noval_LeftStick_X_ID_11978 = false;
	LeftStick_X_ID_11978 = LeftStick_XIn_ID_11995 * STICK_INPUT_SCALING();
	
	noval_LeftStick_Y_ID_14673 = false;
	LeftStick_Y_ID_14673 = -LeftStick_YIn_ID_14666 * STICK_INPUT_SCALING();
	
	noval_RightStick_X_ID_11983 = false;
	RightStick_X_ID_11983 = RightStick_XIn_ID_12000 * STICK_INPUT_SCALING();
	
	noval_RightStick_Y_ID_11988 = false;
	RightStick_Y_ID_11988 = RightStick_YIn_ID_12005 * STICK_INPUT_SCALING();
	
	noval_ButtonL2_ID_14960 = false;
	ButtonL2_ID_14960 = ButtonL2In_ID_14976 * Z_INPUT_SCALING();
	
	noval_ButtonR2_ID_14967 = false;
	ButtonR2_ID_14967 = ButtonR2In_ID_14983 * Z_INPUT_SCALING();
	
	return;
	
}

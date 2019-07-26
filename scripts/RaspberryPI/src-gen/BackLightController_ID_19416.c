/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:02 CEST 2019 */
#include "inc-gen/BackLightController_ID_19416.h"

GEN_TYPE_double BrakePedal_ID_19466;

GEN_TYPE_boolean noval_BrakePedal_ID_19466;

GEN_TYPE_RGBColor BlinkLeft_ID_19539;

GEN_TYPE_boolean noval_BlinkLeft_ID_19539;

GEN_TYPE_RGBColor BlinkRight_ID_19544;

GEN_TYPE_boolean noval_BlinkRight_ID_19544;

GEN_TYPE_RGBColor ColorBackLeft_ID_19483;

GEN_TYPE_boolean noval_ColorBackLeft_ID_19483;

GEN_TYPE_RGBColor ColorBackRight_ID_19488;

GEN_TYPE_boolean noval_ColorBackRight_ID_19488;

void clear_inputs_BackLightController_ID_19416(){
	noval_BrakePedal_ID_19466 = true;
	noval_BlinkLeft_ID_19539 = true;
	noval_BlinkRight_ID_19544 = true;
}

void clear_outputs_BackLightController_ID_19416(){
	noval_ColorBackLeft_ID_19483 = true;
	noval_ColorBackRight_ID_19488 = true;
}

void init_BackLightController_ID_19416(){
	noval_ColorBackLeft_ID_19483 = true;
	noval_ColorBackRight_ID_19488 = true;
}

void perform_step_BackLightController_ID_19416(){
	clear_outputs_BackLightController_ID_19416();
	if (BrakePedal_ID_19466 > 0) {
		noval_ColorBackLeft_ID_19483 = false;
		ColorBackLeft_ID_19483.R = 255;
		ColorBackLeft_ID_19483.B = 0;
		ColorBackLeft_ID_19483.G = 0;
		
		
		noval_ColorBackRight_ID_19488 = false;
		ColorBackRight_ID_19488.R = 255;
		ColorBackRight_ID_19488.B = 0;
		ColorBackRight_ID_19488.G = 0;
		
		
	}
	else {
		noval_ColorBackLeft_ID_19483 = noval_BlinkLeft_ID_19539;
		if (!noval_BlinkLeft_ID_19539) {
			ColorBackLeft_ID_19483 = BlinkLeft_ID_19539;
		}
		
		noval_ColorBackRight_ID_19488 = noval_BlinkRight_ID_19544;
		if (!noval_BlinkRight_ID_19544) {
			ColorBackRight_ID_19488 = BlinkRight_ID_19544;
		}
		
	}
	return;
	
}

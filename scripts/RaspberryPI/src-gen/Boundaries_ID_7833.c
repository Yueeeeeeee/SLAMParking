/* generated by AutoFOCUS 3 (2.14.0) on Tue Jul 09 17:20:03 CEST 2019 */
#include "inc-gen/Boundaries_ID_7833.h"

GEN_TYPE_double Velocity_ID_7870;

GEN_TYPE_boolean noval_Velocity_ID_7870;

GEN_TYPE_double MaxVelocityDiff_ID_7879;

GEN_TYPE_boolean noval_MaxVelocityDiff_ID_7879;

GEN_TYPE_double MaxValue_ID_7840;

GEN_TYPE_boolean noval_MaxValue_ID_7840;

GEN_TYPE_double MinValue_ID_7847;

GEN_TYPE_boolean noval_MinValue_ID_7847;

GEN_TYPE_double MaxErrorSum_ID_7854;

GEN_TYPE_boolean noval_MaxErrorSum_ID_7854;

void clear_inputs_Boundaries_ID_7833(){
	noval_Velocity_ID_7870 = true;
	noval_MaxVelocityDiff_ID_7879 = true;
}

void clear_outputs_Boundaries_ID_7833(){
	noval_MaxValue_ID_7840 = true;
	noval_MinValue_ID_7847 = true;
	noval_MaxErrorSum_ID_7854 = true;
}

void init_Boundaries_ID_7833(){
	noval_MaxValue_ID_7840 = true;
	noval_MinValue_ID_7847 = true;
	noval_MaxErrorSum_ID_7854 = true;
}

void perform_step_Boundaries_ID_7833(){
	clear_outputs_Boundaries_ID_7833();
	if (!(noval_Velocity_ID_7870 == true)) {
		if (!(noval_Velocity_ID_7870 == true) && !(noval_MaxVelocityDiff_ID_7879 == true)) {
			noval_MaxValue_ID_7840 = false;
			MaxValue_ID_7840 = min((Velocity_ID_7870 + myabs(MaxVelocityDiff_ID_7879)), MAX_END_SPEED());
			
			noval_MinValue_ID_7847 = false;
			MinValue_ID_7847 = Velocity_ID_7870 - myabs(MaxVelocityDiff_ID_7879);
			
		}
	}
	else {
		noval_MaxValue_ID_7840 = false;
		MaxValue_ID_7840 = 1.0;
		
		noval_MinValue_ID_7847 = false;
		MinValue_ID_7847 = -1.0;
		
	}
	noval_MaxErrorSum_ID_7854 = false;
	MaxErrorSum_ID_7854 = COEFFICIENT_CONTROLLER_MAX_ERROR_SUM();
	
	return;
	
}

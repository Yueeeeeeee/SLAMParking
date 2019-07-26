/* generated by AutoFOCUS 3 (2.14.0) on Fri May 03 16:10:07 CEST 2019 */
#include "inc-gen/Velocity_of_Obstacle_in_front_ID_6981.h"

GEN_TYPE_double VelocityDiff_ID_7693;

GEN_TYPE_boolean noval_VelocityDiff_ID_7693;

GEN_TYPE_double Velocity_ID_8817;

GEN_TYPE_boolean noval_Velocity_ID_8817;

GEN_TYPE_double LastVelocityIn_ID_7749;

GEN_TYPE_boolean noval_LastVelocityIn_ID_7749;

GEN_TYPE_boolean InvalidValue_ID_7762;

GEN_TYPE_boolean noval_InvalidValue_ID_7762;

GEN_TYPE_double VelocityFrontObstacle_ID_44668;

GEN_TYPE_boolean noval_VelocityFrontObstacle_ID_44668;

GEN_TYPE_double LastVelocityOut_ID_7738;

GEN_TYPE_boolean noval_LastVelocityOut_ID_7738;

void clear_inputs_Velocity_of_Obstacle_in_front_ID_6981(){
	noval_VelocityDiff_ID_7693 = true;
	noval_Velocity_ID_8817 = true;
	noval_LastVelocityIn_ID_7749 = true;
	noval_InvalidValue_ID_7762 = true;
}

void clear_outputs_Velocity_of_Obstacle_in_front_ID_6981(){
	noval_VelocityFrontObstacle_ID_44668 = true;
	noval_LastVelocityOut_ID_7738 = true;
}

void init_Velocity_of_Obstacle_in_front_ID_6981(){
	noval_VelocityFrontObstacle_ID_44668 = true;
	noval_LastVelocityOut_ID_7738 = true;
}

void perform_step_Velocity_of_Obstacle_in_front_ID_6981(){
	clear_outputs_Velocity_of_Obstacle_in_front_ID_6981();
	if (!(noval_InvalidValue_ID_7762 == true) && InvalidValue_ID_7762) {
		noval_VelocityFrontObstacle_ID_44668 = noval_LastVelocityIn_ID_7749;
		if (!noval_LastVelocityIn_ID_7749) {
			VelocityFrontObstacle_ID_44668 = LastVelocityIn_ID_7749;
		}
		
		noval_LastVelocityOut_ID_7738 = noval_LastVelocityIn_ID_7749;
		if (!noval_LastVelocityIn_ID_7749) {
			LastVelocityOut_ID_7738 = LastVelocityIn_ID_7749;
		}
		
	}
	else {
		if (!(noval_Velocity_ID_8817 == true) && !(noval_VelocityDiff_ID_7693 == true)) {
			if (myabs((Velocity_ID_8817 + VelocityDiff_ID_7693)) < VELOCITY_DEADZONE()) {
				noval_VelocityFrontObstacle_ID_44668 = false;
				VelocityFrontObstacle_ID_44668 = 0.0;
				
				noval_LastVelocityOut_ID_7738 = false;
				LastVelocityOut_ID_7738 = 0.0;
				
			}
			else {
				if (NO_WRONG_WAY_DRIVER() && Velocity_ID_8817 + VelocityDiff_ID_7693 < 0.0) {
					noval_VelocityFrontObstacle_ID_44668 = false;
					VelocityFrontObstacle_ID_44668 = 0.0;
					
					noval_LastVelocityOut_ID_7738 = false;
					LastVelocityOut_ID_7738 = 0.0;
					
				}
				else {
					noval_VelocityFrontObstacle_ID_44668 = false;
					VelocityFrontObstacle_ID_44668 = Velocity_ID_8817 + VelocityDiff_ID_7693;
					
					noval_LastVelocityOut_ID_7738 = false;
					LastVelocityOut_ID_7738 = Velocity_ID_8817 + VelocityDiff_ID_7693;
					
				}
			}
		}
	}
	return;
	
}
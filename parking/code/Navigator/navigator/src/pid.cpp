#include "pid.h"

PID::PID(double kp, double ti, double kd){
	this->kp = kp;
	this->ti = ti;
	this->kd = kd;
	this->integral = 0;
	this->previous_error = 0;
}

double PID::step(double error, double timestep, double limit){
	//P
	double P_out = this->kp * error;

	//I
	double new_integral = this->integral + (error * timestep);
	double I_out = (1 / this->ti) * this->integral;

	//D
	double derivative = (error - this->previous_error) / timestep;
	double D_out = this->kd * derivative;
	this->previous_error = error;

	double out = P_out + I_out + D_out;

	if(out > limit) {
		out = limit;
		// Do not integrate if limited.
	}
	else if(out < -limit) {
		out = -limit;
		// Do not integrate if limited.
	}
	else {
		this->integral = new_integral;
	}

	return out;
}

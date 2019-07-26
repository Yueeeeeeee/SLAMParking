//
// Created by Jannik Peters on 13.06.2019.
//

#ifndef PID_H
#define PID_H

class PID {
private:
	double kp;
	double ti;
	double kd;
	double integral;
	double previous_error;

public:
	PID(double kp, double ti, double kd);
	double step(double error, double timestep, double limit);
};

#endif /* PID_H */

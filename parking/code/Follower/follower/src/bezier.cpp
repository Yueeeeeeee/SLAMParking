//
// Created by Jannik Peters on 22.05.2019.
//

#include "bezier.h"

#include <math.h>

double limit(double min, double val, double max) {
    if(val < min) {
        return min;
    }
    else if (val > max) {
        return max;
    }
    else{
        return val;
    }
}

CubicBezierCurve::CubicBezierCurve() {
    this->len = -1.0;
}

CubicBezierCurve::CubicBezierCurve(Vec2f p1, Vec2f p2, Vec2f p3, Vec2f p4) {
    this->start = p1;
    this->waypoint1 = p2;
    this->waypoint2 = p3;
    this->end = p4;
    this->len = -1.0;
}

Vec2f CubicBezierCurve::positionAt(double tau) {
    tau = limit(0.0, tau, 1.0);
    return start     * pow(1 - tau, 3)
         + waypoint1 * (3 * tau * pow(1 - tau, 2))
         + waypoint2 * (3 * pow(tau, 2) * (1 - tau))
         + end       * pow(tau, 3);
}

Vec2f CubicBezierCurve::derivativeAt(double tau) {
    tau = limit(0.0, tau, 1.0);
    double tau2 = tau * tau;
    return start     * (-3.0 * tau2 +  6.0 * tau - 3.0)
         + waypoint1 * ( 9.0 * tau2 - 12.0 * tau + 3.0)
         + waypoint2 * (-9.0 * tau2 +  6.0 * tau)
         + end       * ( 3.0 * tau2);
}

Vec2f CubicBezierCurve::tangentAt(double tau) {
    return derivativeAt(tau).normalize();
}

double CubicBezierCurve::length() {
    if(len == -1.0) {
        double sum = 0.0;
        const int num_steps = 10000;
        const double step_size = 1.0 / num_steps;
        for(int i = 0; i < num_steps; i++) {
            double tau = i * step_size;
            Vec2f step = derivativeAt(tau) * step_size;
            sum += step.length();
        }
        len = sum;
    }
    return len;
}

double CubicBezierCurve::arclength2tau(double arc, double tauBegin, double tauStep) {
    double tau = tauBegin;
    double dist = 0.0;
    while(dist < arc) {
        if(tau >= 1.0) {
            break;
        }
        Vec2f step = derivativeAt(tau) * tauStep;
        dist += step.length();
        tau += tauStep;
    }
    return tau;
}
//
// Created by Jannik Peters on 22.05.2019.
//

#ifndef BEZIER_H
#define BEZIER_H

#include "vector.h"

const double DEFAULT_STEP_WIDTH = 0.0001;

class CubicBezierCurve {
private:
    Vec2f start;
    Vec2f waypoint1;
    Vec2f waypoint2;
    Vec2f end;
    double len;
public:
    CubicBezierCurve();
    CubicBezierCurve(Vec2f p1, Vec2f p2, Vec2f p3, Vec2f p4);
    Vec2f positionAt(double tau);
    Vec2f derivativeAt(double tau);
    Vec2f tangentAt(double tau);
    double arclength2tau(double arc, double tauBegin = 0.0, double tauStep = DEFAULT_STEP_WIDTH);
    double length();
};

#endif //BEZIER_H

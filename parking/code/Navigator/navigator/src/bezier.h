//
// Created by Jannik Peters on 22.05.2019.
//

#ifndef BEZIER_H
#define BEZIER_H

#include "vec2f.h"

const double DEFAULT_STEP_WIDTH = 0.0001;

enum Direction {
    forward, backward
};

class CubicBezierCurve {
private:
    Vec2f start;
    Vec2f waypoint1;
    Vec2f waypoint2;
    Vec2f end;
    double len;
    Direction dir;
public:
    CubicBezierCurve();
    CubicBezierCurve(Vec2f p1, Vec2f p2, Vec2f p3, Vec2f p4, Direction dir);
    Vec2f positionAt(double tau);
    Vec2f derivativeAt(double tau);
    Vec2f tangentAt(double tau);
    double arclength2tau(double arc, double tauBegin = 0.0, double tauStep = DEFAULT_STEP_WIDTH);
    double length();
    Direction direction();
    void setDirection(Direction newdir);
    CubicBezierCurve transform(Vec2f translation, Vec2f orientation, double scale);
    void print();
};

#endif //BEZIER_H

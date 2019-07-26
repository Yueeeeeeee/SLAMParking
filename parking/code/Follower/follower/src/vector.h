//
// Created by Jannik Peters on 22.05.2019.
//

#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>

class Vec2f {
public:
    double x;
    double y;

    Vec2f() : x(0.0), y(0.0) {}

    Vec2f(double x, double y) {
        this->x = x;
        this->y = y;
    }

    double length();
    double lengthSquared();
    Vec2f normalize();
    double dot(Vec2f& other);
    double angle(Vec2f& other);
    Vec2f operator +(const Vec2f&);
    Vec2f operator -(const Vec2f&);
    Vec2f operator *(double);
    Vec2f rotate(const double);

    friend std::ostream& operator<<(std::ostream& os, const Vec2f& vec);
};

#endif //VECTOR_H

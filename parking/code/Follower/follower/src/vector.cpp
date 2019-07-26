//
// Created by Jannik Peters on 22.05.2019.
//

#include <math.h>

#include "vector.h"

using std::ostream;

double Vec2f::length() {
    return sqrt(x * x + y * y);
}

double Vec2f::lengthSquared() {
    return x * x + y * y;
}

Vec2f Vec2f::normalize() {
    double len = length();
    return len == 0.0 ? Vec2f(0.0, 0.0) : Vec2f(x / len, y / len);
}

double Vec2f::dot(Vec2f& other) {
    return this->x * other.x + this->y * other.y;
}

double Vec2f::angle(Vec2f& other) {
	double rawAngle = acos(this->dot(other));
	double side = this->x * other.y - this->y * other.x;
	if(side >= 0.0) {
		return rawAngle;
	}
	else{
		return -rawAngle;
	}
}

Vec2f Vec2f::operator+(const Vec2f & that) {
    return {this->x + that.x, this->y + that.y};
}

Vec2f Vec2f::operator-(const Vec2f & that) {
    return {this->x - that.x, this->y - that.y};
}

Vec2f Vec2f::operator*(const double scalar) {
    return {this->x * scalar, this->y * scalar};
}

Vec2f Vec2f::rotate(const double angle) {
    Vec2f ret;
    ret.x = this->x * cos(angle) - this->y * sin(angle);
    ret.y = this->x * sin(angle) + this->y * cos(angle);
    return ret;
}

std::ostream& operator<<(std::ostream& os, const Vec2f& vec) {
    return os << '(' << vec.x << "; " << vec.y << ')';
}

//
// Created by Jannik Peters on 12.07.2019.
//

#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include <signal.h>
#include <vector>

#include "vec2f.h"
#include "bezier.h"

#include <sstream>

std::vector<CubicBezierCurve> planPath(Vec2f startPosition, Vec2f startOrientation, Vec2f goalPosition, Vec2f goalOrientation);

#endif // PLANNER_H
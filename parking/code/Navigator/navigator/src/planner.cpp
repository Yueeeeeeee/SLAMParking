#include "planner.h"

// For debugging
#include "ros/ros.h"

const double WHEEL_BASE = 0.34;
const double MAX_CURVE_ANGLE = 45;
const double RAD_TO_DEG = 180.0/3.141593;
const double D = 0.8;
const double VEC_LEN = 0.45;

struct Task {
    Vec2f startPosition;
    Vec2f startOrientation;
    Vec2f goalPosition;
    Vec2f goalOrientation;
    double scale;
};

enum GoalPosition {N, NE, E, SE, S, SW, W, NW};
enum GoalOrientation {n, e, s, w};
double theta;

//orientate coordinate system so that start position is at (0,0) and start orientation is at (0,1)
Task rotateShift (Task task) {
    Task converted;
    Vec2f desiredStartO(0.0, 1.0);
    Vec2f start(0.0, 0.0);

    converted.startPosition = start;
    converted.startOrientation = desiredStartO;
   
    Vec2f startOnorm = task.startOrientation.normalize();

    // construct a rotation matrix
    Vec2f mr1(startOnorm.y, -startOnorm.x);
    Vec2f mr2(startOnorm.x,  startOnorm.y);

    // poor man's matrix multiplication
    converted.goalOrientation.x = mr1.dot(task.goalOrientation);
    converted.goalOrientation.y = mr2.dot(task.goalOrientation);

    Vec2f goalShifted = task.goalPosition - task.startPosition;
    converted.scale = goalShifted.length();
    converted.goalPosition.x = mr1.dot(goalShifted);
    converted.goalPosition.y = mr2.dot(goalShifted);

    converted.startPosition    = converted.startPosition * (1.0 / converted.scale);
    converted.startOrientation = converted.startOrientation * VEC_LEN;
    converted.goalPosition     = converted.goalPosition * (1.0 / converted.scale);
    converted.goalOrientation  = converted.goalOrientation.normalize() * VEC_LEN;

    //--> now inputsPoints are shifted and rotated so that starting position is at (0,0) 
    // and starting orientation is upwards (0,1)
    return converted;
}

//function that checks if curviture of chosen path exceeds a maximum value
bool isCurvitureOk (std::vector<CubicBezierCurve>& curve, double maxCurveAngle, double tauStep) {
    double tau1 = 0;
    for (int i = 0; i < curve.size(); i++) {
        for (double tau2 = tau1 + tauStep; tau2 <= 1; tau2 = tau2 + tauStep) {
            Vec2f firstTangent = curve.at(i).tangentAt(tau1);
            Vec2f secondTangent = curve.at(i).tangentAt(tau2);
            if (secondTangent.angle(firstTangent) > maxCurveAngle) {
                return false;
            } else {
                tau1 = tau2;
            }
        }
    }
    return true;
}

Vec2f cartesianToPolar (double x, double y) {
    double r = sqrt((x*x) + (y*y));
    double t = atan2(y, x) * RAD_TO_DEG;
    if(t < 0.0) {
        t += 360.0;
    }
    Vec2f goalPolar(r, t);
    return goalPolar;
}

void constructSingleSpline(Task task, std::vector<CubicBezierCurve>& path) {
    CubicBezierCurve curve(
        task.startPosition,
        task.startPosition + task.startOrientation,
        task.goalPosition - task.goalOrientation,
        task.goalPosition,
        Direction::forward
    );
    path.push_back(curve);
}

void constructSingleSplineBackward(Task task, std::vector<CubicBezierCurve>& path) {
    CubicBezierCurve curve(
        task.startPosition,
        task.startPosition - task.startOrientation,
        task.goalPosition + task.goalOrientation,
        task.goalPosition,
        Direction::backward
    );
    path.push_back(curve);
}

void constructDualSpline(
    Task task, 
    Vec2f intermediatePosition, 
    Vec2f intermediateOrientation, 
    std::vector<CubicBezierCurve>& path)
{
    CubicBezierCurve curve1(
        task.startPosition,
        task.startPosition + task.startOrientation,
        intermediatePosition - intermediateOrientation,
        intermediatePosition,
        Direction::forward
    );
    CubicBezierCurve curve2(
        intermediatePosition,
        intermediatePosition - intermediateOrientation,
        task.goalPosition + task.goalOrientation,
        task.goalPosition,
        Direction::backward
    );
    
    path.push_back(curve1);
    path.push_back(curve2);
}

void constructDualSplineBackward(
    Task task, 
    Vec2f intermediatePosition, 
    Vec2f intermediateOrientation, 
    std::vector<CubicBezierCurve>& path)
{
    CubicBezierCurve curve1(
        task.startPosition,
        task.startPosition - task.startOrientation,
        intermediatePosition + intermediateOrientation,
        intermediatePosition,
        Direction::backward
    );
    CubicBezierCurve curve2(
        intermediatePosition,
        intermediatePosition + intermediateOrientation,
        task.goalPosition - task.goalOrientation,
        task.goalPosition,
        Direction::forward
    );
    
    path.push_back(curve1);
    path.push_back(curve2);
}

//functions for five different paths leading to N, NE/NW, E/W, SE/SW, S
std::vector<CubicBezierCurve> pathToN (Task task, GoalOrientation gO) {
    std::vector<CubicBezierCurve> path;
    if (gO == n) {
        // single spline
        constructSingleSpline(task, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s) {
        //add control point to the right
        Vec2f intermediatePosition(D, task.goalPosition.y / 2.0);
        Vec2f intermediateOrientation(VEC_LEN, 0.0);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {   //curviture is too large--> add another control point and move the first one
            //...
        } 
    } else if (gO == e) {
        //add control point to the right
        Vec2f intermediatePosition(task.goalPosition.x + D, task.goalPosition.y);
        Vec2f intermediateOrientation(VEC_LEN, 0.0);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == w) {
        Vec2f intermediatePosition(task.goalPosition.x - D, task.goalPosition.y);
        Vec2f intermediateOrientation(-VEC_LEN, 0.0);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    }
    return path;
}

std::vector<CubicBezierCurve> pathToNE (Task task, GoalOrientation gO, double mirror) {
    std::vector<CubicBezierCurve> path;
    if (gO == n) {
        Vec2f intermediatePosition(mirror * task.goalPosition.x / 2.0, 2.0);
        Vec2f intermediateOrientation(0.0, VEC_LEN * 2.0);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s) {
        Vec2f intermediatePosition(task.goalPosition.x, task.goalPosition.y - D);
        Vec2f intermediateOrientation(0.0, VEC_LEN);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == e) {
        //no control points
        constructSingleSpline(task, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == w) {
        Vec2f intermediatePosition(mirror * -D, task.goalPosition.y);
        Vec2f intermediateOrientation(-VEC_LEN * mirror, 0.0);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    }
    return path;
}

std::vector<CubicBezierCurve> pathToE (Task task, GoalOrientation gO, double mirror) {
    std::vector<CubicBezierCurve> path;
    if (gO == n) {
        Vec2f intermediatePosition(task.goalPosition.x / 2.0, 2.0);
        Vec2f intermediateOrientation(0.0, VEC_LEN * 2.0);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s || gO == e) {
        //no control points
        constructSingleSpline(task, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == w) {
        Vec2f intermediatePosition(mirror * -D, D);
        Vec2f intermediateOrientation(-VEC_LEN * mirror, 0.0);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    }
    return path;
}

std::vector<CubicBezierCurve> pathToSE (Task task, GoalOrientation gO, double mirror) {
    std::vector<CubicBezierCurve> path;
    if (gO == n) {
        Vec2f intermediatePosition(mirror * task.goalPosition.x / 2.0, 2.0);
        Vec2f intermediateOrientation(0.0, VEC_LEN * 2.0);
        constructDualSpline(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == w) {
        //no control points
        constructSingleSplineBackward(task, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s) {
        //no control points
        constructSingleSpline(task, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == e) {
        Vec2f intermediatePosition(mirror * (-D), task.goalPosition.y);
        Vec2f intermediateOrientation(VEC_LEN, 0.0);
        constructDualSplineBackward(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    }
    return path;
}

std::vector<CubicBezierCurve> pathToS (Task task, GoalOrientation gO) {
    std::vector<CubicBezierCurve> path;
    if (gO == n) {
        //no control points
        constructSingleSplineBackward(task, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s) {      //exact same as NS !!
        Vec2f intermediatePosition(D, (task.goalPosition.y)/2);
        Vec2f intermediateOrientation(-VEC_LEN, 0.0);
        constructDualSplineBackward(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //...
        }
    } else if (gO == e) {
        Vec2f intermediatePosition(-D, task.goalPosition.y);
        Vec2f intermediateOrientation(VEC_LEN, 0.0);
        constructDualSplineBackward(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == w) {
        Vec2f intermediatePosition(D, task.goalPosition.y);
        Vec2f intermediateOrientation(-VEC_LEN, 0.0);
        constructDualSplineBackward(task, intermediatePosition, intermediateOrientation, path);
        if (!isCurvitureOk(path, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    }
    return path;
}

GoalOrientation mirrorGoalOrientation(GoalOrientation in) {
    if (in == w) {
        return e;
    }
    else if (in == e) {
        return w;
    }
    else { // n or s
        return in;
    }
}

std::vector<CubicBezierCurve> planPath(Vec2f startPosition, Vec2f startOrientation, Vec2f goalPosition, Vec2f goalOrientation) {
    std::vector<CubicBezierCurve> list;

    //1) from the given vector array, determine scenario
    //2) from the scenario, determine control points

    //1) determine scenario:
    //1.1) what is the direction of the goal position with respect to the starting position/orientation
    //polar coordinates--> angles
    //convert goal coordinates to polar coordinates

    Task input;
    input.startPosition = startPosition;
    input.startOrientation = startOrientation;
    input.goalPosition = goalPosition;
    input.goalOrientation = goalOrientation;

    Task converted = rotateShift(input);

    Vec2f goalPolar = cartesianToPolar(converted.goalPosition.x, converted.goalPosition.y);

    GoalPosition gP;
    if (goalPolar.y < 22.5) {
        gP = E;
        ROS_INFO("Goal in the east.");
    } else if (goalPolar.y < 67.5) {
        gP = NE;
        ROS_INFO("Goal in the north east.");
    } else if (goalPolar.y < 112.5) {
        gP = N;
        ROS_INFO("Goal in the north.");
    } else if (goalPolar.y < 157.5) {
        gP = NW;
        ROS_INFO("Goal in the north west.");
    } else if (goalPolar.y < 202.5) {
        gP = W;
        ROS_INFO("Goal in the west.");
    } else if (goalPolar.y < 247.5) {
        gP = SW;
        ROS_INFO("Goal in the south west.");
    } else if (goalPolar.y < 292.5) {
        gP = S;
        ROS_INFO("Goal in the south.");
    } else if (goalPolar.y < 337.5) {
        gP = SE;
        ROS_INFO("Goal in the south east.");
    } else if (goalPolar.y < 360.0) {
        gP = E;
        ROS_INFO("Goal in the east.");
    }

    //1.2) what is goal orientation with respect to starting orientation
    //does goalO have to be normalized?--> yes
    //anges in radiant
    GoalOrientation gO;

    double orientationAngle = converted.startOrientation.angle(converted.goalOrientation) * RAD_TO_DEG;
    if (orientationAngle > 135 || orientationAngle < -135) {
        gO = s;
        ROS_INFO("Goal orientated to south.");
    } else if (orientationAngle > 45) {
        gO = w;
        ROS_INFO("Goal orientated to west.");
    } else if (orientationAngle > -45) {
        gO = n;
        ROS_INFO("Goal orientated to north.");
    } else if (orientationAngle > -135) {
        gO = e;
        ROS_INFO("Goal orientated to east.");
    }

    switch (gP) {
        case N:
            list = pathToN(converted, gO);
            break;
        case NE:
            list = pathToNE(converted, gO, 1.0);
            break;
        case E:
            list = pathToE(converted, gO, 1.0);
            break;
        case SE:
            list = pathToSE(converted, gO, 1.0);
            break;
        case S:
            list = pathToS(converted, gO);
            break;
        case NW:
            //mirror goalOrientation vertically
            gO = mirrorGoalOrientation(gO);
            list = pathToNE(converted, gO, -1.0);
            break;
        case W:
            //mirror goalOrientation vertically
            gO = mirrorGoalOrientation(gO);
            list = pathToE(converted, gO, -1.0);
            break;
        case SW:
            //mirror goalOrientation vertically
            gO = mirrorGoalOrientation(gO);
            list = pathToSE(converted, gO, -1.0);
            break;
    }

    std::vector<CubicBezierCurve> transformed;
    for(int i=0;i<list.size();i++) {
        CubicBezierCurve localCurve = list.at(i);
        ROS_INFO("Spline no. %d", i);
        localCurve.print();
        CubicBezierCurve globalCurve = localCurve.transform(startPosition, startOrientation, converted.scale);
        transformed.push_back(globalCurve);
    }
    return transformed;
}

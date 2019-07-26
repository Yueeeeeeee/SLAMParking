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

//#include "ros/ros.h"
//#include "std_msgs/Float32.h"
//#include "std_msgs/UInt8.h"

#include "vector.h"
#include "bezier.h"
//#include "velocity_profile.h"

#include <sstream>

const uint8_t STATE_MANUAL = 0;
const uint8_t STATE_PARKING = 1;

const int OUTPUT_BUFFER_SIZE = 3;
const double WHEEL_BASE = 0.34;
const double MAX_CURVE_ANGLE = 45;

volatile unsigned char state = STATE_PARKING;
//volatile float actualVelocity = 0.0f;
Vec2f i1(5.0, 5.0);  //start
Vec2f i2(4.0, 7.0);    //start orientation
Vec2f i3(7.0, -11.0);     //goal  orientation
Vec2f i4(7.0, -10.0);    //goal
Vec2f inputPoints[4] = {i1, i2, i3, i4}; //input values which are the start/goal position and orientation stored in an array of vectors
Vec2f controlPoints[4]; //max two control points but every control point also needs an orientation point
Vec2f p1[4];
Vec2f p2[4];
Vec2f p3[4];
Vec2f p4[4];
CubicBezierCurve c(p1[0], p2[0], p3[0], p4[0]);
CubicBezierCurve curve[3] = {c, c, c};
//x-coordinates of control points (max 2--> in total max 4 because of additional control orientation)
//index 0-> orientation, index1-> control point, etc
double x[4];
double y[4];

const double toDegrees = 180.0/3.141593;

enum goalPosition {N, NE, E, SE, S, SW, W, NW};
goalPosition gP;
enum goalOrientation {n, e, s, w};
goalOrientation gO;
//int d = 5; //arbitrary distance in which control points are set

//orientate coordinate system so that starting position is at (0,0)

Vec2f desiredStartO(0.0, 1.0);
Vec2f start(0.0, 0.0);
Vec2f startOshifted(inputPoints[1].x - inputPoints[0].x, inputPoints[1].y - inputPoints[0].y);
Vec2f startOnorm = startOshifted.normalize();

double theta = desiredStartO.angle(startOnorm); //rotation angle


Vec2f startO = desiredStartO;
//shift goal orientation and position
Vec2f goalOshifted(inputPoints[2].x - inputPoints[0].x, inputPoints[2].y - inputPoints[0].y);
Vec2f goalshifted(inputPoints[3].x - inputPoints[0].x, inputPoints[3].y - inputPoints[0].y);
//rotate goal orientation and position
Vec2f goalO(goalOshifted.x * cos(-theta) - goalOshifted.y * sin(-theta), goalOshifted.x * sin(-theta) + goalOshifted.y * cos(-theta));
Vec2f goal(goalshifted.x * cos(-theta) - goalshifted.y * sin(-theta), goalshifted.x * sin(-theta) + goalshifted.y * cos(-theta));
//--> now inputsPoints are shifted and rotated so that starting position is at (0,0) and starting orientation is upwards (0,1)

Vec2f goalPolar;
Vec2f goalOShifted1(goalO.x - goal.x, goalO.y - goal.y);
Vec2f goalOnorm = goalOShifted1.normalize();

volatile bool terminate = false;

//void stateCallback(const std_msgs::UInt8::ConstPtr& msg);
//void pathCallback(const std_msgs::Float32::ConstPtr& msg);

void intHandler(int dummy) 
{
    terminate = true;
}
//function that checks if curviture of chosen path exceeds a maximum value
bool isCurvitureOk (CubicBezierCurve curve[], double maxCurveAngle, double tauStep) {
    double tau1 = 0;
    for (int i = 0; i < curve->length(); i++) {
        for (double tau2 = tau1 + tauStep; tau2 <= 1; tau2 = tau2 + tauStep) {
            Vec2f firstTangent = curve[i].tangentAt(tau1);
            Vec2f secondTangent = curve[i].tangentAt(tau2);
            if (secondTangent.angle(firstTangent) > maxCurveAngle) {
                return false;
            } else {
                tau1 = tau2;
            }
        }
    }
    return true;
}

//sets the x and y coordinates of the control points at the positions given as parameter
void setCoordinates (double distanceX, double distanceY, double deltaX, double deltaY) {
    //control point orientation
    x[0] = start.x + distanceX + deltaX; 
    y[0] = start.y + distanceY + deltaY;
    //control point
    x[1] = start.x + distanceX;     
    y[1] = start.y + distanceY;
}
void setControlPoints (int numberOfPoints, double x[], double y[]) {
    for (int i = 0; i < numberOfPoints; i++) {
        controlPoints[i].x = x[i];
        controlPoints[i].y = y[i];
    }
}
void setTotalPointsTwoSplines (Vec2f start, Vec2f startO, Vec2f goal, Vec2f goalO, Vec2f controlPoints[]) {
    p1[0] = start;
    p1[1] = controlPoints[1];
    p2[0] = startO;
    p2[1] = controlPoints[0];
    p3[0] = controlPoints[0];
    p3[1] = goalO;
    p4[0] = controlPoints[1];
    p4[0] = goal;
}
void setTotalPointsOneSplineInvertedGoal (Vec2f start, Vec2f startO, Vec2f goal, Vec2f goalO) {
    p1[0] = start;
    p2[0] = startO;
    p3[0] = goal;
    p4[0] = goalO;
}
void setTotalPointsOneSplineInvertedStart (Vec2f start, Vec2f startO, Vec2f goal, Vec2f goalO) {
    p1[0] = startO;
    p2[0] = start;
    p3[0] = goalO;
    p4[0] = goal;
}
void setSplines (int numberOfSplines, Vec2f p1[], Vec2f p2[], Vec2f p3[], Vec2f p4[]) {
    for (int i = 0; i < numberOfSplines; i++) {
        CubicBezierCurve c(p1[i], p2[i], p3[i], p4[i]);
        curve[i] = c;
    }
}
//for driving backwards, the orientation has to be flipped
/*Vec2f flipped () {
    //rotate orientation by 180 degrees
    Vec2f flip;
    int rad = deg2rad(180);
    return flip = new Vec2f(this->x * cos(rad) - this->y * sin(rad), this->x * sin(rad) + this->y * cos(rad));
}*/

Vec2f cartesianToPolar (double x, double y) {

    double r = sqrt((pow(x,2))+(pow(y,2)));
    double t = atan(y/x) * toDegrees;
    Vec2f goalPolar(r, t);
    return goalPolar;
}

//functions for five different paths leading to N, NE/NW, E/W, SE/SW, S
void pathToN (Vec2f goal, Vec2f goalO, goalOrientation gO) {
    if (gO == n) {
        //no control points
        setTotalPointsOneSplineInvertedGoal (start, startO, goal, goalO);
        setSplines(1, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s) {
        //add control point to the right
        setCoordinates(5.0, (goal.y - start.y)/2, -1.0, 0.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines (start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {   //curviture is too large--> add another control point and move the first one
            //...
        } 
    } else if (gO == e) {
        setCoordinates(5.0, (-start.y + goal.y), -1.0, 0.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines (start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) { 
            //set additional control points/splines
        }
    } else if (gO == w) {
        setCoordinates(-5.0, (-start.y + goal.y), 1.0, 0.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines (start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) { 
            //set additional control points/splines
        }
    }
}
void pathToNE (Vec2f goal, Vec2f goalO, goalOrientation gO, double distanceX, double deltaX) {
    if (gO == n) {
        //no control points
        setTotalPointsOneSplineInvertedGoal (start, startO, goal, goalO);
        setSplines(1, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s) {
        setCoordinates((-start.x + goal.x), -5.0, 0.0, 1.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines (start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) { 
            //set additional control points/splines
        }
    } else if (gO == e) {
        //no control points
        setTotalPointsOneSplineInvertedGoal (start, startO, goal, goalO);
        setSplines(1, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == w) {      //exact same as NW !!
        setCoordinates(distanceX, (-start.y + goal.y), deltaX, 0.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines(start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    }
}
void pathToE (Vec2f goal, Vec2f goalO, goalOrientation gO, double distanceX, double deltaX) {
    if (gO == n) {
        setCoordinates(0.0, -5.0, 0.0, 1.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines(start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) { 
            //set additional control points/splines
        }
    } else if (gO == s || gO == e) {
        //no control points
        setTotalPointsOneSplineInvertedGoal (start, startO, goal, goalO);
        setSplines(1, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == w) {
        setCoordinates(distanceX, 5.0, deltaX, 0.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines(start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) { 
            //set additional control points/splines
        }
    }
}

void pathToSE (Vec2f goal, Vec2f goalO, goalOrientation gO) {
    if (gO == n || gO == w) {
        //no control points
        setTotalPointsOneSplineInvertedStart (start, startO, goal, goalO);
        setSplines(1, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s || gO == e) {
        //no control points
        setTotalPointsOneSplineInvertedGoal (start, startO, goal, goalO);
        setSplines(1, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    }
}
void pathToS (Vec2f goal, Vec2f goalO, goalOrientation gO) {
    if (gO == n) {
        //no control points
        setTotalPointsOneSplineInvertedStart (start, startO, goal, goalO);
        setSplines(1, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) {
            //set additional control points/splines
        }
    } else if (gO == s) {      //exact same as NS !!
        setCoordinates(5.0, (goal.y - start.y)/2, -1.0, 0.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines(start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) { 
            //...
        }
    } else if (gO == e) {
        setCoordinates(-5.0, (-start.y + goal.y), 1.0, 0.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines(start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) { 
            //set additional control points/splines
        }
    } else if (gO == w) {
        setCoordinates(5.0, (-start.y + goal.y), -1.0, 0.0);
        setControlPoints(2, x, y);
        setTotalPointsTwoSplines(start, startO, goal, goalO, controlPoints);
        setSplines(2, p1, p2, p3, p4);
        if (!isCurvitureOk(curve, MAX_CURVE_ANGLE, 0.0001)) { 
            //set additional control points/splines
        }
    }
}

int main(int argc, char **argv) {

	/*ros::init(argc, argv, "pathPlanner");
    ros::NodeHandle n;

	//"path" containing info about start and goal position and orientation
	ros::Subscriber pathSubscriber = n.subscribe("path", 1000, pathCallback);
    ros::Subscriber stateSubscriber = n.subscribe("state", 1000, stateCallback);

    ros::Publisher controlPointPublisher = n.advertise<std_msgs::Float32>("controlPoints", OUTPUT_BUFFER_SIZE);
    std_msgs::Float32 controlPointsMsg;

    ros::Rate loop_rate(100); // [Hz]*/

    
    //orientate coordinate system so that starting position is at (0,0)

    //1) from the given vector array, determine scenario
    //2) from the scenario, determine control points


    signal(SIGINT, intHandler);
    //while(ros::ok() && !terminate) {
        if(state == STATE_MANUAL) {

            //?

        } else if(state == STATE_PARKING) {

            //1) determine scenario:
            //1.1) what is the direction of the goal position with respect to the starting position/orientation
            //polar coordinates--> angles
            //convert goal coordinates to polar coordinates
            goalPolar = cartesianToPolar(goal.x, goal.y);

            if (goalPolar.y < 22.5) {
                gP = E;
            } else if (goalPolar.y < 67.5) {
                gP = NE;
            } else if (goalPolar.y < 112.5) {
                gP = N;
            } else if (goalPolar.y < 157.5) {
                gP = NW;
            } else if (goalPolar.y < 202.5) {
                gP = W;
            } else if (goalPolar.y < 247.5) {
                gP = SW;
            } else if (goalPolar.y < 292.5) {
                gP = S;
            } else if (goalPolar.y < 337.5) {
                gP = SE;
            } else if (goalPolar.y < 360.0) {
                gP = E;
            }

            //1.2) what is goal orientation with respect to starting orientation
            //does goalO have to be normalized?--> yes
            //anges in radiant



            if ((startO.angle(goalOnorm)*toDegrees) > 135 || (startO.angle(goalOnorm)*toDegrees) < -135) {
                gO = s;
            } else if ((startO.angle(goalOnorm)*toDegrees) > 45) {
                gO = w;
            } else if ((startO.angle(goalOnorm)*toDegrees) > -45) {
                gO = n;
            } else if ((startO.angle(goalOnorm)*toDegrees) > -135) {
                gO = e;
            }

            switch (gP) {
                case N: pathToN (goal, goalO, gO);
                        break;
                case NE: pathToNE (goal, goalO, gO, -5.0, 1.0);
                        break;
                case E: pathToE (goal, goalO, gO, -5.0, 1.0);
                        break;
                case SE: pathToSE (goal, goalO, gO);
                        break;
                case S: pathToS (goal, goalO, gO);
                        break;
                case NW: pathToNE (goal, goalO, gO, 5.0, -1.0);
                        break;
                case W: pathToE (goal, goalO, gO, 5.0, -1.0);
                        break;
                case SW: pathToSE (goal, goalO, gO);
                        break;

                }
        }



    //}
        //ros::spinOnce();
        //loop_rate.sleep();

        std::cout << "ControlPoints:" << std::endl;
        for (int i = 0; i < controlPoints->length(); i++) {
            std::cout << controlPoints[i] << std::endl;
        }
        std::cout << "theta: " << theta*toDegrees << std::endl;
        std::cout << "goal Orientation: " << goalO << std::endl;
        std::cout << "goal : " << goal << std::endl;
        std::cout << "goal polar: " << goalPolar << std::endl;
        std::cout << "goal position: " << gP << std::endl;
        std::cout << "start orientation: " << startO << std::endl;
        std::cout << "goal orientation norm: " << goalOnorm << std::endl;
        std::cout << "angle between start and goal orientation: " << startO.angle(goalOnorm)*toDegrees << std::endl;
        std::cout << "goal orientation: " << gO << std::endl;



    return 0;
}




/*void stateCallback(const std_msgs::UInt8::ConstPtr& msg) {
    state = msg->data;
}

void pathCallback(const std_msgs::Float32::ConstPtr& msg) {
    inputPoints = msg->data;    //cannot work because type is array of vectors
    //ROS_INFO("Actual Velocity %f", msg->data);
}*/






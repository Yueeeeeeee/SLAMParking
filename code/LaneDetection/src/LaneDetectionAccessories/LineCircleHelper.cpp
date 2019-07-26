
#include <opencv2/opencv.hpp>
#include<iterator>



using namespace std;
using namespace cv;


namespace lch{

// functions talking about lines here abuse the cv::Point2d to save informations about lines
// if the line is given by the formula y=mx+b
// Point.x is m and Point.y is b
Point2d leastSquaresLine(vector<Point2i> &spline, double &error);
Point2d leastSquaresLine(vector<Point2d> &spline, double &error);
int lineXforY(Point2d &line,int y);
double distanceLineToPoint(double x, double y, Point2d &line);
// functions talking about circles here abuse the cv::Point3d to save informations about circles
// Point.x is x position of circle-center
// Point.y is y position of circle-center
// Point.z is circle radius
Point3d leastSquaresCircle(std::vector<Point2d> &points);

//code implements formulas from this site
//https://www.varsitytutors.com/hotmath/hotmath_help/topics/line-of-best-fit

Point2d leastSquaresLine(vector<Point2d> &spline, double &error){
		double x_mean=0.0, y_mean=0.0;
		for (Point2d &p : spline) {
			x_mean += p.x;
			y_mean += p.y;
		}

		x_mean /= spline.size();
		y_mean /= spline.size();
		double num=0.0,denom=0.0;
		for (Point2d &p : spline) {
			num += (p.x - x_mean) * (p.y - y_mean);
			denom += (p.x - x_mean) * (p.x - x_mean);
		}
		if (denom == 0.0) {
			denom = 0.01;
			//this is a hack because otherwise division by zero might happen.
			//better solution: use different mathematical representation for lines than y= mx + b
		}
		double m = num / denom;
		double b = y_mean - (m * x_mean);

		//calc error sum (distance point to line)
		error=0.0;
		Point2d resultLine(m,b);
		for (Point2d &p : spline) {
			error += distanceLineToPoint(p.x,p.y,resultLine);
		}

		return resultLine;

}

Point2d leastSquaresLine(vector<Point2i> &spline, double &error) {
	vector<Point2d> doublePoints;
	for(Point2i &p:spline){
		doublePoints.push_back(p);
	}
	return leastSquaresLine(doublePoints,error);
}

int lineXforY(Point2d &line,int y){
	return (int)((y-line.y)/line.x);
}

double distanceLineToPoint(double x, double y, Point2d &line){

	return abs(line.y + line.x * x - y) / sqrt(1 + line.x * line.x);
}

//code implements formulas from this pdf
//https://dtcenter.org/met/users/docs/write_ups/circle_fit.pdf
Point3d leastSquaresCircle(std::vector<Point2d> &points) {
	double x_mean = 0.0;
	double y_mean = 0.0;
	for (Point2d &point : points) {
		x_mean += point.x;
		y_mean += point.y;
	}
	x_mean /= points.size();
	y_mean /= points.size();
	double suu=0.0, suv=0.0, svv=0.0, suuu=0.0, svvv=0.0, suvv=0.0, svuu=0.0;

	for (Point2d &point : points) {
		double xtemp = point.x - x_mean;
		double ytemp = point.y - y_mean;
		suu += xtemp * xtemp;
		suv += xtemp * ytemp;
		svv += ytemp * ytemp;
		suuu += (xtemp * xtemp * xtemp);
		svvv += (ytemp * ytemp * ytemp);
		suvv += (xtemp * ytemp * ytemp);
		svuu += (ytemp * xtemp * xtemp);
	}

	Mat_<double> A = (Mat1d(2, 2) << suu, suv, suv, svv);
	Mat_<double> B = (Mat1d(2, 1) << 0.5 * (suuu + suvv), 0.5 * (svvv + svuu));
	Mat_<double> sol = A.inv() * B;

	double uc = sol.at<double>(0, 0);
	double vc = sol.at<double>(0, 1);
	double xc = uc + x_mean;
	double yc = vc + y_mean;

	double alpha = uc * uc + vc * vc + ((suu + svv) / points.size());
	double radius = sqrt(alpha);
	return Point3d(xc, yc, radius);
}

}

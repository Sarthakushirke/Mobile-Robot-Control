#include <math.h>
#include <iostream>
#include <unistd.h>
#include <vector> 

using namespace std;

class VFH{
private:
int nbins = 0;
double angle_min = 0;
double angle_max = 0;
double range_min = 0;
double range_max = 0;
double increment = 0;
int nbeams = 10;

double max_window = 2.0;
double min_window = 0.01;

double pi = 3.14159265359;
double thresh=20; // for map 3 use 17, for map 4 use 19 (not robust tho)
double robot_radius = 0.3; 
  
// Vector from the origin to the laser end point (h, k)
double dx;
double dy;

// Vector from the origin to the circle center (x, y)
double fx;
double fy;

// Variables for solving intersection points
double a;
double b;
double c;
double discriminant;

public:
int removed = 0;
void makeHistogram(std::vector<double>& SD);
double getClosestFreeAngle(double AA, double GA);
double AngletoGoal(double x, double y, double gx, double gy);
double Histogram[100];
void hitCircles(std::vector<double>& SD);
double hitCoord[2][1000];
double altSD[1000];
void setMinimum(double angle_min, double angle_max, double range_min, double range_max, double increment);
// std::vector<std::pair<double, double>> findLineCircleIntersections(double h, double k, double x, double y, double r); 
};


#include <math.h>
#include <vector>
#include "./DataTypes.h"


double V(coordinates X, coordinates G, double k);

coordinates dV(coordinates X, coordinates G, double k);

double Vab(coordinates X, coordinates P);

double dVab(double x, double p);

double S(double Vab);

double dS(double Vab);


std::pair<double, double> velocity_finder(coordinates Pose, double orient, coordinates goalPose, std::vector<coordinates> hitPoints);

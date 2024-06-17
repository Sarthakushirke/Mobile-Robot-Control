#include <math.h>
#include "./DataTypes.h"


//Tools for computations and Map handles





double edge_dist(Node a, Node b);


double wrapToPi(double angle);


coordinates bot_to_map(double dist, double angle, coordinates mapBot);

coordinates sim_world_to_map(coordinates Pose);


coordinates sim_map_to_world(coordinates Pose);


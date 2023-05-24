

#include <vector>
#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>


using namespace std;


// Values
int option = 1;
bool debug = false;
bool debugObstacleDetector = true;

bool wheelsleep = false; // turning uncertaint odometry


// init total static


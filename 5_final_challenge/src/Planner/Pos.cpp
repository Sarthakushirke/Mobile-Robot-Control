#include "../../include/Planner/Pos.h"
#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> 
#include <algorithm>
#include "tools.h"
#include <math.h>

using namespace std;

void Position::initPosition(double initX, double initY, double initA)
{
    x = initX;
    y = initY;
    a = initA;
}

void Position::PosFromOdom(double rx, double ry, double ra)
{
    a = wrapToPi(a + ra);
    x -=sin(a)*ry;
    x +=cos(a)*rx;
    y +=sin(a)*rx;
    y +=cos(a)*ry;
}

double Position::getA()
{
return a;
}

double Position::getX()
{
return x;   
}

double Position::getY()
{
return y;    
}


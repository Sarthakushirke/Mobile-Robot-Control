#include "../../include/Planner/VFH.h"
#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> 
#include <algorithm>

#define MAX_BAR_HEIGHT 10
#define BAR_CHAR '#'
#define NUM_BINS 10


void VFH::hitCircles(std::vector<double>& SD)
{
  for(int i=0; i<SD.size(); i++)
  {
    hitCoord[0][i] = sin((i - 2) * 0.004) * SD[i];
    hitCoord[1][i] = cos((i - 2) * 0.004) * SD[i];
  }
}

std::vector<std::pair<double, double>> findLineCircleIntersections(double h, double k, double x, double y, double r) 
{
    std::vector<std::pair<double, double>> intersections;
    // Vector from the origin to the laser end point (h, k)
    double dx = h;
    double dy = k;
    
    // Vector from the origin to the circle center (x, y)
    double fx = x;
    double fy = y;

    double a = dx * dx + dy * dy;
    double b = - 2 * (fx * dx + fy * dy);
    double c = fx * fx + fy * fy - r * r;

    double discriminant = b * b - 4 * a * c;

    // If the discriminant is negative, there is no intersection
    if (discriminant < 0) {
        return intersections; // Empty vector, no intersection
    } else {
        // Calculate the two potential intersection points
        discriminant = std::sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        // Check if the intersection points are within the segment [0,1]
        if (t1 >= 0 && t1 <= 1) {
            double intersectX1 = t1 * dx;
            double intersectY1 = t1 * dy;
            intersections.push_back(std::make_pair(intersectX1, intersectY1));
        }

        if (t2 >= 0 && t2 <= 1) {
            double intersectX2 = t2 * dx;
            double intersectY2 = t2 * dy;
            intersections.push_back(std::make_pair(intersectX2, intersectY2));
        }
    }

    return intersections;
}

void VFH::makeHistogram(std::vector<double>& SD)
{  
    // std::cout<<"Angle min is " <<angle_min<<std::endl;
    // std::cout<<"Angle max is " <<angle_max<<std::endl;
    // std::cout<<"Range min is " <<range_min<<std::endl;
    // std::cout<<"Range max is " <<range_max<<std::endl;
    // std::cout<<"Increment is " <<increment<<std::endl;
  nbins = SD.size()/10;
  double UHG[nbins]; // 100
  double window = SD.size() / 2 + 1; // 251 for 1000 SD

  for (int l=0+(window-1)/2; l<(SD.size()-(window-1)/2);l++)
  {
    double altDist = 0;
    double minValue = max_window;
    for (int c=0; c<window; c++)
    {
      std::vector<std::pair<double, double>> intersections  = findLineCircleIntersections(hitCoord[0][l], hitCoord[1][l], hitCoord[0][l+c-int((window-1)/2)], hitCoord[1][l+c-int((window-1)/2)], robot_radius);
      if (!intersections.empty()) 
      {
        for (const auto& point : intersections) 
        {
          altDist = sqrt(point.first * point.first + point.second * point.second); 
          if (altDist < minValue)
          {
            minValue = altDist;
          }
        }

        if (minValue < SD[l])
        {
          altSD[l] = minValue;
        }
        else if (minValue >= SD[l])
        {
          altSD[l] = SD[l];
        }
      } 
    }
  }

  std::cout<<"SD[500] : " << SD[SD.size()/2] << endl;
  std::cout<<"altSD[500] : " << altSD[SD.size()/2] << endl;

  for(int m=0;m<nbins;m++)
  {
    Histogram[m]=0;
    for(int n = 0;n<nbeams;n++)
    {
      double distance = altSD[n+m*nbeams];
      if(distance >= max_window){distance = max_window+10;}
      else if(distance <= min_window) {distance = min_window;}

      UHG[m] += (1/distance);
    }
  }

  

  // for(int i=10;i<90;i++)
  // {
  //   if(UHG[i]>thresh)
  //   {
  //     UHG[i] += 0.8;
  //     for(int j=0;j<11;j++)
  //     {
  //       // Histogram[i+(j-5)] += 0.8;
  //       Histogram[i+(j-10)] += UHG[i]+0.8;
  //     }
  //   }
  // }


  for(int i=0;i<nbins;i++)
  {
    Histogram[i] += UHG[i];
  }

}

double VFH::getClosestFreeAngle(double AA, double GA)
{
  double smallestA= 100;
  int index=0;

  for(int i=0;i<nbins;i++)
  {
    double tempangle = (i-(nbins+removed)/2)*(nbeams)*increment;    //(i-50) * 10 * 0.004
    // double tempangle =
    double dAngle = abs(GA-(tempangle+AA));

    if(dAngle < smallestA && Histogram[i] < thresh)
    {
      smallestA = dAngle;
      index = i;
    }
  
  }
  return AA+((index-(nbins+removed)/2)*nbeams*increment);
}
    

  

double VFH::AngletoGoal(double x, double y, double gx, double gy)
{
double dx = gx-x;
double dy = gy-y;
double angle = atan2(dy,dx);
return angle;
}

void VFH::setMinimum(double ang_min, double ang_max, double ran_min, double ran_max, double ang_increment)
{
    angle_min = ang_min;
    angle_max = ang_max;
    range_min = ran_min;
    range_max = ran_max;
    increment = ang_increment;
}

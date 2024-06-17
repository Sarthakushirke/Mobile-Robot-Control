#include <emc/io.h>
#include <emc/rate.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <cmath>
#include "Planner/VFH.h"
#include "Planner/Pos.h"
#include "Planner/planning.h"
#include "PRM/prm.h"
#include "tools.h"
using namespace std;

int main()
{
  //std::cout<<std::setprecision(2);

  emc::Rate r(10);  
  emc::IO io;
  emc::LaserData scan;
  emc::OdometryData odom;
  VFH LN;
  Position robot;

  int nlasers = 0;
  while(io.ok())
  {
    // Read laser data once to get the number of lasers
    if (io.readLaserData(scan))
    {
        nlasers = scan.ranges.size();
        LN.setMinimum(scan.angle_min, scan.angle_max, scan.range_min, scan.range_max, scan.angle_increment);
        std::cout << "nlasers is " << nlasers << std::endl;
        break;
    }
    else
    {
        std::cerr << "Failed to read initial laser data." << std::endl;
        // continue; // If no laser data is read, continue the loop
    }

    if (nlasers == 0)
    {
        std::cerr << "No laser data available, nlasers is 0." << std::endl;
        // continue;
    }
  }

  while(io.ok())
  {
    std::vector<double> scanData(nlasers, 0.0); // Initialize the vector with the size of nlasers

    if(io.readLaserData(scan))
    {
      for(int i = 0; i < nlasers; i++)
      {
        scanData[i] = scan.ranges[i];
      }
      int lastDigit = nlasers % 10;

      if (lastDigit != 0) {
        std::cout<<"Size of old scandata is " << scanData.size() <<std::endl;
        int newSize = nlasers - lastDigit;
        scanData.resize(newSize);
        LN.removed = lastDigit;
        }
      std::cout<<"Size of new scandata is " << scanData.size() <<std::endl;
    }

    if(io.readOdometryData(odom))
    {
      robot.PosFromOdom(odom.x, odom.y, odom.a);
    }

    LN.hitCircles(scanData);
    LN.makeHistogram(scanData);

    double angle = LN.getClosestFreeAngle(robot.getA(), LN.AngletoGoal(robot.getX(), robot.getY(), 6.0, 0.0));

    io.sendBaseReference(0.15, 0.0, (robot.getA() - angle) * -2);

    r.sleep();
  }

  return 0;
}

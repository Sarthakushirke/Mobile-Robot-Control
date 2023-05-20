#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>

int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;
    // Create Rate object, which will help keeping the loop at a fixed frequency
    emc::Rate r(10);
    
    // Variables to store previous odometry data
    emc::OdometryData prevOdom;
    bool isFirstLoop = true;

    // Loop while we are properly connected
    while (io.ok())
    {
        // Create objects to hold the laser and odometry data
        emc::LaserData scan;
        emc::OdometryData odom;
        
        // Send a reference to the base controller (vx, vy, vtheta)
        io.sendBaseReference(0.1, 0, 0);

        if (io.readOdometryData(odom))
        {
            // Print the odometry
            std::cout << "Current time step odomentry: " << odom.timestamp << "\n"
                      << "Current x odomentry: " << odom.x << "\n"
                      << "Current y odomentry: " << odom.y << "\n"
                      << "Current a rotation odomentry: " << odom.a << std::endl;

            // Calculate difference only from the second loop
            if (!isFirstLoop)
            {
                double deltaX = odom.x - prevOdom.x;
                double deltaY = odom.y - prevOdom.y;
                double deltaA = odom.a - prevOdom.a;

                std::cout << "Delta x: " << deltaX << "\n"
                          << "Delta y: " << deltaY << "\n"
                          << "Delta a: " << deltaA << std::endl;
            }
            
            // Store the current odometry as the previous odometry for the next loop
            prevOdom = odom;
            isFirstLoop = false;
        }
        else
        {
            std::cout << "Failed to read odom data" << std::endl;
        }

        // Sleep for the remaining time
        r.sleep();
    }

    return 0;
}

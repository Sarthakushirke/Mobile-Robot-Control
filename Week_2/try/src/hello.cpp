#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>
#include <cmath>

inline float getAngle(float increment, int i) 
{
        return increment * i;
}


float getBeta(float angleIncrement, int beginIdx, int endIdx, const std::vector<float>& ranges) {
    const float obstacleAngleBegin = getAngle(angleIncrement, beginIdx);
    const float obstacleAngleEnd = getAngle(angleIncrement, endIdx);
    const float obstacleAngle = std::abs(obstacleAngleEnd - obstacleAngleBegin);

    const float distanceBC = std::sqrt(
        ranges[beginIdx] * ranges[beginIdx] +
        ranges[endIdx] * ranges[endIdx] -
        2 * ranges[endIdx] * ranges[beginIdx] * std::cos(obstacleAngle)
    );
    const double beta = std::acos(
        (distanceBC * distanceBC + ranges[beginIdx] * ranges[beginIdx] - ranges[endIdx] * ranges[endIdx]) /
        (2 * ranges[beginIdx] * distanceBC)
    );

    return static_cast<float>(beta); // return in degrees
}


int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;
    // Create Rate object, which will help keeping the loop at a fixed frequency
    emc::Rate r(10);
    
    // Variables to store previous odometry data
    double currentAngle = 0.0;
    double previousAngle = 0.0;

    std::array<int, 2> angle_beta;

    // Loop while we are properly connected
    while (io.ok())
    {
        // Create objects to hold the laser and odometry data
        emc::LaserData scan;      
        const float minimum_threshold = 1.2;   // Minimum threshold for obstacle detection

       
        if (io.readLaserData(scan))
        {
            const double forward_angle = M_PI_4;		   // 45 degrees
			const double angle_increment = scan.angle_increment;

			// Calculate the range of indices to search for obstacles
			const int range_search = static_cast<int>(forward_angle / angle_increment);   //195.6 DEGREES
			const int start_index = (scan.ranges.size() / 2) - range_search;  //500 -195.6 = 303
			const int end_index = (scan.ranges.size() / 2) + range_search;    //500 + 195.6 = 696
    
            if(scan.ranges[start_index] < minimum_threshold){
                previousAngle = getBeta(angle_increment,start_index, start_index+1,scan.ranges) ;
            }

            for (int i = start_index; i <= end_index; i++)
			{
                const float cur_range = scan.ranges[i]; 
                if(cur_range < minimum_threshold)
                {   
                    io.sendBaseReference(0,0,0);
                    currentAngle = getBeta(angle_increment,start_index,i+2 ,scan.ranges);
                    if(previousAngle>currentAngle)
                    {                        
                        std::cout << "Rotate anticlockwise" << std::endl;
                        io.sendBaseReference(0,0,-2);
                    }
                    else if(currentAngle>previousAngle)
                    {
                        std::cout << "Rotate clockwise" << std::endl;
                        io.sendBaseReference(0,0,2);
                    }
                    // else if (currentAngle == previousAngle) 
                    // {
                    //     io.sendBaseReference(0,0,0);
                    // }   
                }
                else{
                    // Send a reference to the base controller (vx, vy, vtheta)
                    io.sendBaseReference(0.1, 0, 0);
                }
                // Update previous angle with the current angle
                previousAngle = currentAngle;
            }
        }
        else
        {
            std::cout << "Failed to read scan data" << std::endl;
        }

        // Sleep for the remaining time
        r.sleep();
    }

    return 0;
}

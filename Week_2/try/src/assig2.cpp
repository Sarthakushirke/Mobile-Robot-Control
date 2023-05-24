#include <emc/io.h>
#include <emc/rate.h>
#include <iostream>
#include <algorithm>
#include <cmath>

inline float getAngle(float increment, int i) 
{
        return increment * i;
}

float getDistance(float angleIncrement, int beginIdx, int endIdx, const std::vector<float>& ranges) {
    const float obstacleAngleBegin = getAngle(angleIncrement, beginIdx);
    const float obstacleAngleEnd = getAngle(angleIncrement, endIdx);
    const float obstacleAngle = std::abs(obstacleAngleEnd - obstacleAngleBegin);

    const float distanceBC = std::sqrt(
        ranges[beginIdx] * ranges[beginIdx] +
        ranges[endIdx] * ranges[endIdx] -
        2 * ranges[endIdx] * ranges[beginIdx] * std::cos(obstacleAngle)
    );
    return static_cast<float>(distanceBC); 
}

float getBeta(float angleIncrement, int beginIdx, int endIdx, const std::vector<float>& ranges) {
    const float obstacleAngleBegin = getAngle(angleIncrement, beginIdx);
    const float obstacleAngleEnd = getAngle(angleIncrement, endIdx);
    const float obstacleAngle = std::abs(obstacleAngleEnd - obstacleAngleBegin);

    float distanceBC = getDistance(angleIncrement,beginIdx, endIdx, ranges);

    const double beta = std::acos(
        (distanceBC * distanceBC + ranges[beginIdx] * ranges[beginIdx] - ranges[endIdx] * ranges[endIdx]) /
        (2 * ranges[beginIdx] * distanceBC)
    );
    return static_cast<float>(beta); 
}

float getArea(float angleIncrement,int beginIdx, int endIdx, const std::vector<float>& ranges)
{
    float distanceBC = getDistance(angleIncrement,beginIdx, endIdx, ranges);
    float angle_beta = getBeta(angleIncrement,beginIdx,endIdx,ranges);
    float cal_area =  0.5*distanceBC*ranges[beginIdx]* std::sin(angle_beta);
    return static_cast<float>(cal_area);
}

int main()
{
    // Create IO object, which will initialize the io layer
    emc::IO io;
    // Create Rate object, which will help keeping the loop at a fixed frequency
    emc::Rate r(10);
    
    // Variables to store previous odometry data
    //float obstacle_Area = 0.0;
    float open_area = 0.0;

    // Initialize the maximum open area
    float maxOpenArea = 0.0;

    // middle of the max open area
    int middle_openArea = 0.0;

    //Angle of the middle_openArea id 
    float middle_openArea_angle = 0.0;


    // middle of the robot angle
    float middle_robot_angle = 0.0;

    //Tolerance 
    int tolerance = 0.1;
    //rotation speed 
    const double rotationSpeed = 0.5;  // Adjust the rotation speed as needed

    int id_reached = 0;


    std::vector<std::pair<float, std::pair<int, int>>> areas_with_indices;  // Container to store the pairs of first_i, last_i values and open_area

    // Loop while we are properly connected
    while (io.ok())
    {
        // Create objects to hold the laser and odometry data
        emc::LaserData scan;  
        emc::OdometryData odom;    
        const float minimum_threshold = 0.7;   // Minimum threshold for obstacle detection

        if (io.readLaserData(scan))
        {
            const double forward_angle = M_PI_4;		   // 45 degrees
			const double angle_increment = scan.angle_increment;

			// Calculate the range of indices to search for obstacles
			const int range_search = static_cast<int>(forward_angle / angle_increment);   //195.6 DEGREES
			const int start_index = (scan.ranges.size() / 2) - range_search;  //500 -195.6 = 303
			const int end_index = (scan.ranges.size() / 2) + range_search;    //500 + 195.6 = 696
    
            int first_i = -1;  // Initialize with a value that indicates no i has entered the else statement yet
            int last_i = -1;   // Initialize with a value that indicates no i has entered the else statement yet

            int maxFirst_i = -1;
            int maxLast_i = -1;

            int middle_robot = 0.5*(start_index + end_index);
            std::cout << "middle_robot: " << middle_robot << std::endl;


            for (int i = start_index; i <= end_index; i++)
			{
                const float cur_range = scan.ranges[i]; 
                if(cur_range < minimum_threshold)
                {   
                    io.sendBaseReference(0,0,0);

                    //obstacle_Area =  obstacle_Area + getArea(angle_increment, i ,i+1 ,scan.ranges); 
                    // Reset first_i and last_i if an i enters the if statement
                    first_i = -1;
                    last_i = -1;
                }
                else
                {
                    io.sendBaseReference(0,0,0);
                    open_area = open_area + getArea(angle_increment,i , i+1 ,scan.ranges);
                    //std::cout << " Open Area: " << open_area << std::endl;
                    // Send a reference to the base controller (vx, vy, vtheta)
                    //io.sendBaseReference(0.1, 0, 0);
                    // Update first_i and last_i when an i enters the else statement for the first time
                    if (first_i == -1)
                    {
                        first_i = i;
                        last_i = i;
                    }
                    // Update last_i to keep track of the latest i that enters the else statement
                    else
                    {
                        last_i = i;
                        // Store the pair of first_i and last_i values
                        areas_with_indices.push_back(std::make_pair(open_area, std::make_pair(first_i, last_i))); 
                    }
                    //std::cout << " FIrst_i: " << first_i << std::endl;
                    //std::cout << " Last_i: " << last_i << std::endl;
                } 
            }
            // Go towards open area 
            // Iterate over the areas_with_indices vector
            for (const auto& areaWithIndices : areas_with_indices) 
            {
                // Access the open area and its corresponding first_i and last_i
                double openArea = areaWithIndices.first;
                int first_i = areaWithIndices.second.first;
                int last_i = areaWithIndices.second.second;
               
                // Update the maximum open area and its corresponding indices if a larger one is found
                if (openArea > maxOpenArea) {

                    maxOpenArea = openArea;
                    maxFirst_i = first_i;
                    maxLast_i = last_i;
                }
                //std::cout << "Maximum Open Area: " << maxOpenArea << std::endl;
            }
            
            // Check if a maximum open area was found
            if (maxOpenArea > 0.0) 
            {
                // Print the maximum open area and its corresponding indices
                std::cout << "Maximum Open Area: " << maxOpenArea << std::endl;
                std::cout << "First_i: " << maxFirst_i << std::endl;
                std::cout << "Last_i: " << maxLast_i << std::endl;
                
                middle_openArea = 0.5*(maxFirst_i + maxLast_i) ;
                std::cout << "middle open area " << middle_openArea << std::endl;
                //id_reached = middle_robot;
                // Calculate the error as the difference between desired and current middle position
                middle_openArea_angle = scan.ranges[middle_openArea];
                std::cout << "middle open angle " << middle_openArea_angle << std::endl;
                middle_robot_angle = scan.ranges[middle_robot]; 
                std::cout << "middle robot  angle" << middle_robot_angle << std::endl;
                float error = middle_openArea_angle- middle_robot_angle;
                std::cout << "error: " << error << std::endl;

                while (std::abs(middle_robot_angle - middle_openArea_angle) > tolerance)
                {
                    //I entered the while loop 
                    if (io.readOdometryData(odom))
                    {
                        double currentodomangle = odom.a;
                        std::cout << "current odom angle " << currentodomangle << std::endl;

                        // Rotate the robot based on the error (proportional control)
                        float rotation = rotationSpeed * error;
                        std::cout << "rotation: " << rotation << std::endl;

                        // Send the rotation command to the robot (e.g., using io.sendBaseReference())
                        if(error > 0)
                        {
                            std::cout << "Robot rotated" << std::endl;
                            io.sendBaseReference(0,0,rotation);
                            double rotatedodomangle = odom.a;
                            std::cout << "rotatedodomangle" << rotatedodomangle << std::endl;
                            double actual_angle = currentodomangle - rotatedodomangle;
                            std::cout << "actual_angle" << actual_angle << std::endl;
                            error = error - actual_angle;
                            
                            //int n_id = actual_angle/angle_increment;
                            //std::cout << "n_id " << n_id << std::endl;
                            //int id_reached = middle_robot + n_id;
                        }
                        else
                        {
                            io.sendBaseReference(0,0,-rotation);
                            double rotatedodomangle = odom.a;
                            double actual_angle = std::abs(currentodomangle - rotatedodomangle);
                            error = error + actual_angle;
                            //int actual_angle = rotatedodomangle - currentodomangle;
                            //float n_id = actual_angle/angle_increment;
                            //int id_reached = middle_robot - n_id;
                        }
                    }
                }
                //move untill the maxopen area is not having any colisions 
                for (int i = maxFirst_i; i <= maxLast_i; i++)
                {
                    const float curr_range = scan.ranges[i]; 
                    if(curr_range > minimum_threshold)
                    {
                        io.sendBaseReference(0.1,0,0);
                    }
                
                }

            }
            else 
            {
                // Handle the case when areas_with_indices is empty
                std::cout << "No open areas found" << std::endl;
            }

            areas_with_indices.clear();

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


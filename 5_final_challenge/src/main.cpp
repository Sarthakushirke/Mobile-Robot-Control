#include <cmath>
#include "Planner/planning.h"
#include "PRM/prm.h"
#include "tools.h"
#include <emc/io.h>
#include <emc/rate.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include "Planner/VFH.h"
#include "Planner/Pos.h"
using namespace std;

int main(int argc, char** argv)
{
    /*Initialize file name and constants*/
    std::string filename;
    std::cout << "Using User-provided config-file: ";
    filename = argv[1];
    std::cout << filename << std::endl;

    double cycle_freq = 20.0;
    double init_waiting_rate = 2.0;
    double distance_threshold = 0.4; // 0.05 [m]
    double err_a_threshold = 0.1; // [rad]
    double vel_rotate = 1.5; // [rad/s]
    double vel_translate = 2.5; // [m/s]
    double gain_rotate_while_translate = 5;
    std::array<double, 3UL> draw_path_color = {0.0, 0.0, 1.0};
    double draw_path_width = 0.1;

    /* Initialization */
    emc::IO emc_io;
    emc::LaserData scan;
    emc::OdometryData odom;
    emc_io.readOdometryData(odom);
    emc::Rate r_init_waiting(init_waiting_rate);
    VFH LN;
    Position robot;
    Planner planner;

    // Setting min/max/increment values for use in VFH planner
    int nlasers = 0;
    while (emc_io.ok())
    {
        if (emc_io.readLaserData(scan))
        {
            nlasers = scan.ranges.size();
            LN.setMinimum(scan.angle_min, scan.angle_max, scan.range_min, scan.range_max, scan.angle_increment);
            std::cout << "nlasers is " << nlasers << std::endl;
            break;
        }
        else
        {
            std::cerr << "Failed to read initial laser data." << std::endl;
            continue; // If no laser data is read, continue the loop
        }
        if (nlasers == 0)
        {
            std::cerr << "No laser data available, nlasers is 0." << std::endl;
            continue;
        }
    }
    
    /* Load config file and check if it is meant for a gridmap or PRM */
    auto programConfig = load_config_file(filename);

    /*Localisation (not really, hard coding initial points right now)*/
    double pos_x = 6.0; // in world frame
    double pos_y = 3.5; // in world frame
    double pos_a = 3.14; // in world frame
    robot.initPosition(pos_x, pos_y, pos_a);

    /* Global Planning */
    bool showPRM = true;
    PRM prm;
    prm.generatePRM(programConfig, showPRM);
    coordinates t3(6.3,4.8);
    planner = constructPlannerFromPRM(t3, prm);
    planner.planPath();

    if (!planner._path_found) {
        printf("No path could be found, so I will not try to follow it\n");
        return 0;
    }
    printf("Path from start to goal found, visiting the following nodes: ");
    for (int node_id : planner._path_node_IDs){
        if (node_id != planner._start_nodeID) {
            printf("-");
        }
        printf("%i", node_id);
    }

    /* Convert path node IDs to coordinates */
    std::vector<std::vector<double>> path_coordinates = planner.getPath();

    /* Wait until the path can be sent (when the map has been loaded) */
    r_init_waiting.sleep();
    while(!emc_io.sendPath(path_coordinates, draw_path_color, draw_path_width) && emc_io.ok()){
        r_init_waiting.sleep();
    }

    /*Update waypoint as goal for local planner*/
    int i_next = 0;
    double x_next = path_coordinates[i_next][0]; // in world frame
    double y_next = path_coordinates[i_next][1]; // in world frame
    emc::Rate r(cycle_freq);
    emc_io.speak("I will now follow the path!");


    while (emc_io.ok())
    {
        /*Data processing to avoid segmentation errors when creating bins*/
        std::vector<double> scanData(nlasers, 0.0); 
        if(emc_io.readLaserData(scan))
        {
        for(int i = 0; i < nlasers; i++)
        {
            scanData[i] = scan.ranges[i];
        }
        int lastDigit = nlasers % 10;

        if (lastDigit != 0) {
            // std::cout<<"Size of old scandata is " << scanData.size() <<std::endl;
            int newSize = nlasers - lastDigit;
            scanData.resize(newSize);
            LN.removed = lastDigit;
            }
        // std::cout<<"Size of new scandata is " << scanData.size() <<std::endl;
        }

        /*Update robot position and calculate distance to waypoint*/
        if (emc_io.readOdometryData(odom))
        {
            robot.PosFromOdom(odom.x, odom.y, odom.a);
        }
        std::cout<<"EMC x is " << pos_x << " EMC y is " << pos_y << " EMC a is " << pos_a << std::endl;
        std::cout<<"IO x is " << robot.getX() << " IO y is " << robot.getY() << " IO a is " << robot.getA() <<std::endl;
        

        double err_x = robot.getX() - x_next; // in world frame
        double err_y = robot.getY()- y_next; // in world frame
        double err_a = wrapToPi(robot.getA() - atan2(y_next-pos_y, x_next - pos_x)); // in world frame

        double err_x_next, err_y_next;
        if (i_next < path_coordinates.size() -1)
        {
            err_x_next = robot.getX() - path_coordinates[i_next+1][0]; // in world frame
            err_y_next = robot.getY()- path_coordinates[i_next+1][1]; // in world frame
        }
        
        std::cout<<"Err next is " << err_x_next << " and " << err_y_next << std::endl;
        // Check if target has been reached
        if (err_x*err_x + err_y*err_y <= distance_threshold*distance_threshold)
        {
            // Update waypoint
            i_next++;
            if (i_next >= path_coordinates.size())
                break;
            x_next = path_coordinates[i_next][0];
            y_next = path_coordinates[i_next][1];
        }
        else if (err_x_next*err_x_next + err_y_next*err_y_next <= distance_threshold*distance_threshold)
        {
           i_next++; 
           if (i_next >= path_coordinates.size())
                break;
            x_next = path_coordinates[i_next][0];
            y_next = path_coordinates[i_next][1];
        }

        /*Execute obstacle avoidance*/
        LN.hitCircles(scanData);
        LN.makeHistogram(scanData);
        double angle = LN.getClosestFreeAngle(robot.getA(), LN.AngletoGoal(robot.getX(), robot.getY(), x_next, y_next));

        for(int i = 0; i < nlasers; i++)
        {
            if (scanData[i] < 0.2)
            {
                emc_io.sendBaseReference(0.0, 0.0, (robot.getA() - angle) * -4);
            } 
            else
            {
                emc_io.sendBaseReference(0.15, 0.0, (robot.getA() - angle) * -2);
            }
        }
        printf("target: %f, %f\n", x_next, y_next);

        r.sleep();
    }

    /* Stop the movement of the robot */
    emc_io.sendBaseReference(0.0,0.0,0.0);
    emc_io.speak("I have followed the path");

}
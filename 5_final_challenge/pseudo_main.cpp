// initialize libraries

#include "Planner/planning.h"
#include "PRM/prm.h"
#include "tools.h"
#include <emc/io.h>
#include <emc/rate.h>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include "Planner/VFH.h"
#include "Planner/Pos.h"
using namespace std;


// initialize constants


// while io.ok

//     initialize (emc object, odom object, laserdata, initial coordinates and orientations, table numbers/orders)

//     load config file to import things like table coordinates

//     run localisation function
//     run a few loops to localise/drive into a corner to localise
//     update pose to robot object

//     call global planning function

//     update waypoint as goal for local planner
//     execute obstacle avoidance

//     while current table < last table
//         if statements
//         if (door reached)
//             orient towards door
//             speak to the door ("open please!")
//             if (door opens)
//                 move forward
//             else (door stays closed)
//                 find another

//         if (path is blocked - no path forward that the local planner can find) 
//             if (distance is too close)
//                 emergency stop 
//                 move back map update
//                 redo planning
//             else (distance is above threshold)
//                 move back
//                 map update
//                 redo planning 

//         if (table reached)
//             match orientation of the table 
//             if (orientation fails)
//                 emergency stop
//                 move back
//                 map update
//                 redo planning
//             else (orientation is successful)
//                 order completed (say "i have arrived")
//                 if (order < last order)
//                     current table = next table 
//                 else (order == last order)
//                     say "last order complete"
//                     return 0

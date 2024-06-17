

#include "./open_space.h"
#include "../include/DataTypes.h"

// #include "../include/objects/Robot.h"

using namespace std;


#define NODE_ID_NOT_SET -1
typedef std::chrono::steady_clock::time_point time_point;


double calculate_distance(Grid node_A, Grid node_B)
{
    return sqrt((node_A.mapX-node_B.mapX)*(node_A.mapX-node_B.mapX) + (node_A.mapY-node_B.mapY)*(node_A.mapY-node_B.mapY));
}




bool simulate_path_following(std::vector<Grid> &gridList, std::vector<int> &path_node_IDs, Robot& robot, int tableID)
{
    /* Settings */
    
    int skipCount = 0;

    emc::OdometryData odom;
    
    std::array<double, 3UL> draw_path_color = {0.0, 0.0, 1.0};
    double draw_path_width = 0.1;
    
    int n_points = path_node_IDs.size();

    // Convert path node IDs to coordinates
    std::vector<std::vector<double>> path_coordinates;

    for (int i = 0; i < path_node_IDs.size(); ++i)
    {
        path_coordinates.push_back({ gridList[path_node_IDs[i]].mapX, gridList[path_node_IDs[i]].mapY });
        
    }

    #if SIMULATOR
    robot.draw_io(path_coordinates, draw_path_color, draw_path_width);
    #endif 

    double skip_range = 1; //in meters

    for (int i = 0; i < path_node_IDs.size(); ++i)
    {
        
        if(skipCount < skip_range/gridLength)
        
        {
            int prev_t;
        
           
            if (i == 0)
            {
                prev_t = -1;
            }
            else
                prev_t += 1;

        
            // if (gridList[path_node_IDs[i]].logOdd  0)
            if (i == path_node_IDs.size()-1 ) // Reaching table node
            {
                odom = robot.getOdomData();
                coordinates Pose(odom.x, odom.y);


                if(gridList[path_node_IDs[i]].logOdd < LOG_ODD_THRESH)
                {
                    if(open_space(gridList[path_node_IDs[i]].mapX, gridList[path_node_IDs[i]].mapY, 0.05, robot, prev_t, path_node_IDs[i]))
                    {
                        

                        double goalErr_a = wrapToPi(odom.a - atan2(map_table[tableID].y - Pose.y,  map_table[tableID].x - Pose.x));

                        double angTol = 0.05;

                        bool odomUpdate = false;
                        
                        
                        while(abs(goalErr_a) >= angTol)
                        {
                            
                            if(robot.newOdomData())
                            {
                                odom = robot.getOdomData();
                                odomUpdate = true;
                            }
                            if(odomUpdate)
                            {
                                robot.sendInputs(0, 0, -0.6*goalErr_a/abs(goalErr_a));

                                goalErr_a = wrapToPi(odom.a - atan2(map_table[tableID].y - Pose.y,  map_table[tableID].x - Pose.x));
                            }
                        }
            
                        std::string tableString = "Arrived at Table " + std::to_string(tableID );

                        #if SIMULATOR
                        std::cout << tableString << std::endl;
                        #else
                        robot.voiceCall(tableString);
                        #endif

                        //Stopping the bot for pick up
                        robot.sendInputs(0, 0, 0);
                        auto startTime = std::chrono::steady_clock::now();
                        while(1)
                        {
                            auto currentTime  = std::chrono::steady_clock::now();
                            auto elapsedTime  = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
                            if(elapsedTime == WAIT_TIME)
                                break;

                        }
                        
                        
                        
                        
                        continue;
                    }
                    else 
                    {
                        return false;
                    }

                
                    
                }
                else
                {
                    robot.sendInputs(0,0,0);
                    return false;
                }
            }	

            
            else  // Reaching intermediate nodes
            {
                
                //Objects

                if(gridList[path_node_IDs[i]].logOdd > 0 ) 
                {

                    if(robot.newLaserData() && robot.newOdomData())
                    {
                        odom = robot.getOdomData();
                        RobotStates botState(odom.x, odom.y, odom.a);

                        coordinates Pose(odom.x,odom.y);
                        botState.Pose = sim_map_to_world(Pose);

                        botState.a = odom.a + M_PI/2;
                                    

                        occGridMap(robot, botState);
                    }

                }
                
                if(gridList[path_node_IDs[i]].logOdd < LOG_ODD_THRESH ||  gridList[path_node_IDs[i]].doorFlag == true )
                {
                    //Door opening               
                    
                    if(open_space(gridList[path_node_IDs[i]].mapX, gridList[path_node_IDs[i]].mapY, 0.5, robot, prev_t, path_node_IDs[i]))
                        continue;
                    else
                        return false;

                }
            
                else
                {
                    skipCount++;
                    continue;

                }

            }
        
                
        }
        else
        {
            robot.sendInputs(0,0,0);
            assign_gridID();
            create_connections();
            Makegridlist();
            get_table_connections();
            link_node_to_table();
           
            return false;
            
        }


    }
    
    return true;
    // Stop the movement of the robot
    // emc_io.sendBaseReference(0.0,0.0,0.0);
}



int shortest_table_gridID(std::vector<Grid> table, coordinates Pose)
{
    double min = INFINITY;
    int minID;
    for(int i = 0; i<table.size(); ++i)
    {
        
        double dist = pow((table[i].mapX - Pose.x),2) + pow((table[i].mapY - Pose.y),2);
        if(min > dist)
        {
            min = dist;
            minID = table[i].gridID;
        }
       
        
    }
    return minID;
}


int find_table_nodes(int tables_to_go, coordinates Pose)
{
    vector<int> finish_node_id;

    int finish_node_ID;


    finish_node_ID = shortest_table_gridID(table_grids[tables_to_go], Pose);
    

    return finish_node_ID;
}



bool compute_route( int entrance_node_id, int finish_node_ID, Robot &robot, int tableID)
{
    int n_grid = 0;
    vector<int> new_path_node_IDs;
    bool goal_reached = false;
    int nodeID = finish_node_ID;

    Grid node_goal = gridList[nodeID];
    
    for (int nodeID = 0; nodeID < n_grid; nodeID++)
    {
        gridList[nodeID].g = INFINITY; // cost-to-come from the start to this node
        gridList[nodeID].h = calculate_distance(gridList[nodeID], node_goal); // heuristic cost-to-go from this node to the goal
        gridList[nodeID].f = gridList[nodeID].g + gridList[nodeID].h; // cost-to-come + cost-to-go
        gridList[nodeID].parent_node_ID = NODE_ID_NOT_SET;  // ID of the node from which node you arrived at this node with the lowest cost-to-come
    }
    vector<int> open_grid = {entrance_node_id};
    vector<int> closed_grid = {};
    

    gridList[entrance_node_id].g = 0.0;
    gridList[entrance_node_id].f = gridList[entrance_node_id].g + gridList[entrance_node_id].h;
    
 

   
    while (!goal_reached && !open_grid.empty())
    {
    
        int nodeID_minimum_f = *open_grid.begin();
        //Find the i of the node with minimum f and store it in the variable nodeID_minimum_f
        
        double min_f = gridList[*open_grid.begin()].f;

        for (int i : open_grid) {
            if (gridList[i].f < min_f) 
            {
                min_f = gridList[i].f;
                nodeID_minimum_f = i;
            }
        }

        // The goal is reached (and via the optimal path) when the goal is the open node with minimum f
        if (nodeID_minimum_f == nodeID) 
        {
            goal_reached = true;
        } 
        else if (nodeID_minimum_f == NODE_ID_NOT_SET) 
        {
            break;
        } 
        else 
        {               
            int current_nodeID = nodeID_minimum_f;
            
            //Explore the nodes connected to the new current node if they were not closed yet
            
            for(int i = 0; i< gridList[current_nodeID].connections.size();++i)
            {   

                int child_nodeID = gridList[current_nodeID].connections[i];

                auto in_close = std::find(closed_grid.begin(), closed_grid.end(), child_nodeID);

                auto in_open = std::find(open_grid.begin(), open_grid.end(),child_nodeID);
                
                //Add them to the vector of open nodes when they were not open yet
                
                if(in_close == closed_grid.end()) 
                {
                    if(in_open!=open_grid.end())
                    {
                        double temp_g = gridList[current_nodeID].g + calculate_distance(gridList[current_nodeID],gridList[child_nodeID]);

                        if(temp_g < gridList[child_nodeID].g)
                        {
                            gridList[child_nodeID].g = temp_g;//update g
                            gridList[child_nodeID].f = gridList[child_nodeID].g + gridList[child_nodeID].h;//update f
                            gridList[child_nodeID].parent_node_ID = current_nodeID;//
                        }
                    }
                    
                    else
                    {
                        open_grid.push_back(child_nodeID);
                        gridList[child_nodeID].g = gridList[current_nodeID].g + calculate_distance(gridList[current_nodeID],gridList[child_nodeID]);//calculate g
                        gridList[child_nodeID].f = gridList[child_nodeID].g + gridList[child_nodeID].h;//calculate f
                        gridList[child_nodeID].parent_node_ID = current_nodeID;
                    }
                    
                    
                }   
                
            }
            /*-- End Exercise 2/3 --*/

            // remove the current node from the open vector and add it to the closed vector
            open_grid.erase(std::remove(open_grid.begin(),open_grid.end(),current_nodeID));
            closed_grid.push_back(current_nodeID);
        }
    }

    
    if (goal_reached)
    {
        std::cout << " Found Path! " << std::endl;

        int n = nodeID ;
        while (n != entrance_node_id ) {
            new_path_node_IDs.insert(new_path_node_IDs.begin(), n);
            n = gridList[n].parent_node_ID;
        }
        new_path_node_IDs.insert(new_path_node_IDs.begin(), entrance_node_id);
    
        

        
        if(simulate_path_following(gridList, new_path_node_IDs, robot, tableID) == false) 
        {
            return false;
        }
        else 
        {
            return true;
        }
    }
    else    
    {    
        cout<<"Unable to reach goal"<<endl;

        robot.voiceCall("please clear the way");

        return 0;
    }

   

         

    
}



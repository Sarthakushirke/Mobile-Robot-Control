
#include "./open_space.h"

using namespace std;

#define NODE_ID_NOT_SET -1
typedef std::chrono::steady_clock::time_point time_point;


double calculate_distance(Grid node_A, Grid node_B){
return sqrt((node_A.mapX-node_B.mapX)*(node_A.mapX-node_B.mapX) + (node_A.mapY-node_B.mapY)*(node_A.mapY-node_B.mapY));
}





void simulate_path_following(std::vector<Grid> &gridList, std::vector<int> &path_node_IDs)
{
    /* Settings */
    double cycle_freq = 50.0;
    double init_waiting_rate = 2.0;
    double distance_threshold = 0.03; // [m]
    double err_o_threshold = 0.05; // [rad]
    double vel_rotate = 2.0; // [rad/s]
    double vel_translate = 3.0; // [m/s]
    double gain_rotate_while_translate = 10;
    std::array<double, 3UL> draw_path_color = {0.0, 0.0, 1.0};
    double draw_path_width = 0.1;

    
    // emc_io.readOdometryData(odom);
  
    int n_points = path_node_IDs.size();
    // Convert path node IDs to coordinates
    std::vector<std::vector<double>> path_coordinates;



    /* Move to the locations specified in the path one-by-one */
   // std::cout<<"Curent robot Pose"<<odom.x<< ' '<<odom.y<<std::endl;

    for (auto i_node= path_node_IDs.begin(); i_node != path_node_IDs.end(); ++i_node)
    {
        int currentNodeId = *path_node_IDs.begin();
        if (i_node != path_node_IDs.begin())
            currentNodeId = *std::prev(i_node);

        path_coordinates.push_back({gridList[*i_node].mapX,gridList[*i_node].mapY});
     

        if(*i_node == *std::prev(path_node_IDs.end(), 1))
        {
            open_space(gridList[*i_node].mapX, gridList[*i_node].mapY, 0.05);
            std::cout<<"closing on "<< *i_node<<std::endl;
            
        }
        else
        {
            open_space(gridList[*i_node].mapX, gridList[*i_node].mapY, 0.3);
            std::cout<<"heading to "<< *i_node<<std::endl;
            // emc_io.sendPath(path_coordinates, draw_path_color, draw_path_width);
        }
    }

       

    // Stop the movement of the robot
    // emc_io.sendBaseReference(0.0,0.0,0.0);
}
int main(int argc, char** argv)
{
    read_json();
    
    discretize();

    get_table_connections();

    draw_map();

    getchar();

    assign_gridID();


    create_connections();


    //MRC- env
    // emc::Rate r(5);
    // emc::IO emc_io;
    // Robot robot(&emc_io);

    // emc::LaserData laserdata;
    // emc::OdometryData odomdata;

    // Check if the required number of command-line arguments is provided
    if (argc < 2) {
        std::cout << "Usage: ./path <tables_to_go>" << std::endl;
        return 1;
    }

    int n_grid = 0;
    vector<int> tables_to_go;
    std::vector<int> closed_grid;
    // Parse the command-line arguments and store the finish node IDs in the vector
    for (int i = 1; i < argc; i++) {
        int tableid = std::atoi(argv[i]);
        tables_to_go.push_back(tableid);
    }

    // Print the finish node IDs
    // std::cout << "Tables to go: ";
    // for (int node_id : tables_to_go) {
    //     std::cout << node_id << " ";
    // }
    // std::cout << std::endl;

    //grid table_nodes;

    // Table 0 
    for (int i= 0; i< n_row_grid; ++i)
    {
        for (int j= 0; j< n_col_grid; ++j)
        {
            if (oMap[i][j].tableID == 0)
            {
                table_0.push_back(oMap[i][j]);
               
                n_grid ++;
    
            }
            
        }
    }




    // Table 1
    for (int i= 0; i< n_row_grid; ++i)
    {
        for (int j= 0; j< n_col_grid; ++j)
        {
            if (oMap[i][j].tableID == 1)
            {
                table_1.push_back(oMap[i][j]);
                n_grid ++;
    
            }
            
        }
    }

    // Table 2
    for (int i= 0; i< n_row_grid; ++i)
    {
        for (int j= 0; j< n_col_grid; ++j)
        {
            if (oMap[i][j].tableID == 2)
            {
                table_2.push_back(oMap[i][j]);

                n_grid ++;
    
            }
            
        }
    }

    // Table 3
    for (int i= 0; i< n_row_grid; ++i)
    {
        for (int j= 0; j< n_col_grid; ++j)
        {
            if (oMap[i][j].tableID == 3)
            {
                table_3.push_back(oMap[i][j]);
                n_grid ++;
    
            }
            
        }
    }

    // Table 4
    for (int i= 0; i< n_row_grid; ++i)
    {
        for (int j= 0; j< n_col_grid; ++j)
        {
            if (oMap[i][j].tableID == 4)
            {
                table_4.push_back(oMap[i][j]);
                n_grid ++;
    
            }
            
        }
    }

    vector<int> finish_node_ids;

    for (int number : tables_to_go) {
        int get;
        // Define the sublist for each number
        if(number == 0)
        {
            get = table_0[0].gridID;
            finish_node_ids.push_back(get);
        }
        else if(number == 1)
        {
            get = table_1[0].gridID;
            finish_node_ids.push_back(get);
        } 
        else if(number == 2)
        {
            get = table_2[0].gridID;
            finish_node_ids.push_back(get);
        }
        else if(number == 3)
        {
            get = table_3[0].gridID;
            finish_node_ids.push_back(get);
        }
        else if(number == 4)
        {
            get = table_4[0].gridID;
            finish_node_ids.push_back(get);
        }

    }

   // // Print the finish node IDs
    std::cout << "Finish Node IDs: ";
    for (int table_id : finish_node_ids) {
        std::cout << table_id << " ";
    }
    std::cout << std::endl;

    // Open nodes counter
    for (int i= 0; i< n_row_grid; ++i)
    {
        for (int j= 0; j< n_col_grid; ++j)
        {
            if (oMap[i][j].mapObj == false)
            {
                gridList.push_back(oMap[i][j]);
                //std::cout << " Gridlist" << oMap[i][j].gridID << std::endl;
                n_grid ++;
                
            }
            
        }
    }
    
    int start_x = ceil(6.3/gridLength);
    int start_y = ceil(3.2/gridLength);

    // std::cout << " Grid ID entrance" << oMap[start_y][start_x].gridID << std::endl;
    
    int entrance_node_id = oMap[start_y][start_x].gridID;
    
    // Create a temporary vector to store the node IDs of the current path
    vector<int> new_path_node_IDs = {};
    // Plan a path
    /* 0: Initialization */
    bool goal_reached = false;
    
    for (int node_id : finish_node_ids) 
    {
        
        Grid node_goal = gridList[node_id];
      
        for (int nodeID = 0; nodeID < n_grid; nodeID++){
            gridList[nodeID].g = INFINITY; // cost-to-come from the start to this node
            gridList[nodeID].h = calculate_distance(gridList[nodeID], node_goal); // heuristic cost-to-go from this node to the goal
            gridList[nodeID].f = gridList[nodeID].g + gridList[nodeID].h; // cost-to-come + cost-to-go
            gridList[nodeID].parent_node_ID = NODE_ID_NOT_SET;  // ID of the node from which node you arrived at this node with the lowest cost-to-come
        }
        vector<int> open_grid = {entrance_node_id};
        vector<int> closed_grid = {};
        

        gridList[entrance_node_id].g = 0.0;
        gridList[entrance_node_id].f = gridList[entrance_node_id].g + gridList[entrance_node_id].h;
        
        //cout<<"heading to node: "<<node_id<<endl;
    
        /* 1: Plan path using the A* algorithm with:
        * - cost-to-come g: distance covered from the start to the node
        * - heuristic cost-to-go h: straight line distance from the node to the goal */
        while (!goal_reached && !open_grid.empty())
        {
            /* Expand open node with lowest f (except when it's already the goal):
            * - explore its connected nodes and if the cost-to-come to such a node via this note is lower, update it. Open them if they were not open yet.
            * - close this node */

            // Find the node number from the ones that are open with minimum f
            int nodeID_minimum_f = *open_grid.begin();
            //Find the index of the node with minimum f and store it in the variable nodeID_minimum_f
         
            double min_f = gridList[*open_grid.begin()].f;

            for (int i : open_grid) {
                if (gridList[i].f < min_f) {
                    min_f = gridList[i].f;
                    nodeID_minimum_f = i;
                }
            }

            // The goal is reached (and via the optimal path) when the goal is the open node with minimum f
            if (nodeID_minimum_f == node_id) 
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
                    
                    //Add them to the list of open nodes when they were not open yet
                    
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

                // remove the current node from the open list and add it to the closed list
               open_grid.erase(std::remove(open_grid.begin(), open_grid.end(), current_nodeID), open_grid.end());

                closed_grid.push_back(current_nodeID);
            }
        }

        /* 2: Trace back the optimal path (if the goal could be reached) */
        if (goal_reached)
        {
            std::cout << " Goal reached " << std::endl;
            /*-- Exercise 3/3: Put the node IDs of nodes in the optimal path (from entrance to finish) in the list path_node_IDs --*/
            // Put your code here
            int nodeID = node_id ;
            while (nodeID != entrance_node_id ) {
                new_path_node_IDs.insert(new_path_node_IDs.begin(), nodeID);

                nodeID = gridList[nodeID].parent_node_ID;
            }
           new_path_node_IDs.insert(new_path_node_IDs.begin(), entrance_node_id);

            /*-- End Exercise 3/3 --*/
           
          std::vector<int>::iterator it;
            for (it = new_path_node_IDs.begin(); it != new_path_node_IDs.end(); ++it) 
            {
                cout << *it << endl;
            }

           cout<< std::endl;
            
           simulate_path_following(gridList, new_path_node_IDs); 
            
            new_path_node_IDs.clear();        
        }
        else
            cout<<"Unable to reach goal"<<endl;

        goal_reached = false;
        entrance_node_id = node_id;

         
    }
     
    return 0;
}


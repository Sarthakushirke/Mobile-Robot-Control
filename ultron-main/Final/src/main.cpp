

#include "./A_star.h"



int main(int argc, char** argv)
{
    read_json();
    discretize();



    get_table_connections();

 

    assign_gridID();


    create_connections();


    link_node_to_table();


    Makegridlist();

 
    


    //MRC- env
    emc::IO io;

    emc::Rate r(REFRESH_RATE);
    Robot robot(&io);

    emc::LaserData laserData;
    emc::OdometryData odom;
    bool odomUpdate = false, laserUpdate = false;


    while(!odomUpdate || !laserUpdate)
	{
		
		if(robot.newOdomData())
		{
			odom = robot.getOdomData();
			odomUpdate = true;
		}
		if(robot.newLaserData())
		{
			laserData = robot.getLaserData();
			laserUpdate = true;
		}

	}
    
   


    // Check if the required number of command-line arguments is provided
    if (argc < 2) {
        std::cout << "Usage: ./path <tables_to_go>" << std::endl;
        return 1;
    }

    int n_grid = 0;
    vector<int> tables_to_go;
    std::vector<int> closed_grid;

    int entrance_node_ID, finish_node_ID;
    // Parse the command-line arguments and store the finish node IDs in the vector
    for (int i = 1; i < argc; i++) 
    {
        int tableID = std::atoi(argv[i]);

        odom = robot.getOdomData();

        coordinates mapBot(odom.x, odom.y);
        
        finish_node_ID = find_table_nodes(tableID, mapBot);
      

        coordinates wBot = sim_map_to_world(mapBot);
        
        int new_x = ceil(wBot.x/gridLength);
        int new_y = ceil(wBot.y/gridLength);

        entrance_node_ID = oMap[new_y][new_x].gridID;

        // Plan a path
        if(compute_route(entrance_node_ID, finish_node_ID, robot, tableID) == false)
        {
            i--;
         
            
        }
        else
        {
            std::cout<<"going to next table"<<std::endl;
          
        }

    }

    robot.sendInputs(0,0,0);

    draw_map();
     
    return 0;
}


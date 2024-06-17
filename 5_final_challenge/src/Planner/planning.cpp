#include "./Planner/planning.h"

double calculate_distance(Node node_A, Node node_B){
    return sqrt((node_A.x-node_B.x)*(node_A.x-node_B.x) + (node_A.y-node_B.y)*(node_A.y-node_B.y));
}

void Planner::planPath(){

    printf("Start path planning...\n");
    
    /* 0: Initialization */
    int n_nodes = _nodelist.size();
    Node node_goal = _nodelist[_goal_nodeID];
    for (int nodeID = 0; nodeID < n_nodes; nodeID++){
        _nodelist[nodeID].g = INFINITY; // cost-to-come from the start to this node
        _nodelist[nodeID].h = calculate_distance(_nodelist[nodeID], node_goal); // heuristic cost-to-go from this node to the goal
        _nodelist[nodeID].f = _nodelist[nodeID].g + _nodelist[nodeID].h; // cost-to-come + cost-to-go
        _nodelist[nodeID].parent_node_ID = NODE_ID_NOT_SET;  // ID of the node from which node you arrived at this node with the lowest cost-to-come
    }
    std::list<int> open_nodes = {_start_nodeID};
    std::list<int> closed_nodes = {};
    _nodelist[_start_nodeID].g = 0.0;
    _nodelist[_start_nodeID].f = _nodelist[_start_nodeID].g + _nodelist[_start_nodeID].h;
    bool goal_reached = false;

    printf("Completed initialization\n");
    
    /* 1: Plan path using the A* algorithm with:
     * - cost-to-come g: distance covered from the start to the node
     * - heuristic cost-to-go h: straight line distance from the node to the goal */
    while (!goal_reached && !open_nodes.empty())
    {
        /* Expand open node with lowest f (except when it's already the goal):
         * - explore its connected nodes and if the cost-to-come to such a node via this note is lower, update it. Open them if they were not open yet.
         * - close this node */

        // Find the node number from the ones that are open with minimum f
        int nodeID_minimum_f = NODE_ID_NOT_SET;
        /*-- Exercise A* 1/3: Find the index of the node with minimum f and store it in the variable nodeID_minimum_f --*/
        // Put your code here!
        double f_min = INFINITY;
        for (int i : open_nodes){
            if (_nodelist[i].f<f_min){
                f_min = _nodelist[i].f;
                nodeID_minimum_f = i;
               }
        }
        /*-- End Exercise A* 1/3 --*/

        // The goal is reached (and via the optimal path) when the goal is the open node with minimum f
        if (nodeID_minimum_f == _goal_nodeID) {
            goal_reached = true;
        } else if (nodeID_minimum_f == NODE_ID_NOT_SET) {
            break;
        } else {
            int current_nodeID = nodeID_minimum_f;
            /*-- Exercise A* 2/3: Explore the nodes connected to the new current node if they were not closed yet:
             *                 - Add them to the list of open nodes when they were not open yet.
             *                 - If the cost-to-come to such a node via the current node is lower than it was via another node, update it:
             *                     - update g (and f accordingly)
             *                     - update its parent node (should now become equal to the current node) --*/
            // Put your code here!
            // Explore the nodes to new current node if not closed yet
            for (int connector : _connections[current_nodeID])
            {
                bool no_response = (std::find(closed_nodes.begin(),closed_nodes.end(),connector) == closed_nodes.end());
                bool no_response_open = (std::find(open_nodes.begin(),open_nodes.end(),connector) == open_nodes.end());
                //adding them to the list of open nodes when they are not open
                if (no_response && no_response_open)
                {
                    open_nodes.push_back(connector);
                    _nodelist[connector].g = calculate_distance(_nodelist[connector], _nodelist[current_nodeID]) + _nodelist[current_nodeID].g;
                    _nodelist[connector].f = _nodelist[connector].g + _nodelist[connector].h;
                    _nodelist[connector].parent_node_ID = current_nodeID;
                }
                else if (no_response && !no_response_open)
                {
                    int g_tilda = calculate_distance(_nodelist[connector],_nodelist[current_nodeID]) + _nodelist[current_nodeID].g;
                    int f_tilda = g_tilda + _nodelist[connector].h;
                    int parent_tilda = current_nodeID;
                    if (_nodelist[connector].f >f_tilda)
                    {
                        //updating g,f,parent
                        _nodelist[connector].f = f_tilda;
                        _nodelist[connector].g = g_tilda;
                        _nodelist[connector].parent_node_ID = parent_tilda;
                    }
                    
                }
                
            }
            
            /*-- End Exercise A* 2/3 --*/

            // remove the current node from the open list and add it to the closed list
            open_nodes.remove(current_nodeID);
            closed_nodes.push_back(current_nodeID);
        }
    }

    printf("Finished exploration phase\n");
    
    /* 2: Trace back the optimal path (if the goal could be reached) */
    if (goal_reached){
        std::list<int> path_node_IDs = {};
        
        /*-- Exercise A* 3/3: Put the node IDs of nodes in the optimal path (from start to goal) in the list path_node_IDs --*/
        // Put your code here!
        int nodeID = _goal_nodeID;
        while (nodeID != _start_nodeID) {
            path_node_IDs.push_front(nodeID);
            nodeID = _nodelist[nodeID].parent_node_ID;
        }
        path_node_IDs.push_front(_start_nodeID);
        /*-- End Exercise 3/3 --*/

        /*-- End Exercise A* 3/3 --*/

        setPathNodeIDs(path_node_IDs);
        setPathFound(true);
        // Print the optimal path node IDs
        printf("Optimal path node IDs: ");
        for (int id : path_node_IDs) {
            printf("%d ", id);
        }
        printf("\n");
    }

    printf("Finished path planning\n");
}

std::vector<std::vector<double>> Planner::getPath()
{
    std::vector<std::vector<double>> path_coordinates;
    if (!_path_found)
    {
        printf("Path has not been found (yet)");
        return path_coordinates;
    }
    for (int i_node : _path_node_IDs){
        path_coordinates.push_back({_nodelist[i_node].x, _nodelist[i_node].y});
    }
    return path_coordinates;
}

void Planner::setNodeList(const std::vector<Node> &nodelist){
    _nodelist = nodelist;
}

void Planner::setStart(const int &start_nodeID){
    _start_nodeID = start_nodeID;
}

void Planner::setGoal(const int &goal_nodeID){
    _goal_nodeID = goal_nodeID;
}

void Planner::setConnections(const std::vector<std::vector<int>> &connections){
    _connections = connections;
}

void Planner::setPathFound(const bool &path_found){
    _path_found = path_found;
}

void Planner::setPathNodeIDs(const std::list<int> &path_node_IDs){
    _path_node_IDs = path_node_IDs;
}


Planner constructPlannerFromGridmap(const nlohmann::json &programConfig)
{
    // ------
    // Initialize the Planner
    // ------

    // Default config values, will be overwritten if the config files contains the correct information
    double resolution = 0.5;
    double pixels_per_node = 2;
    int max_nodes_x = 10;
    int max_nodes_y = 10;
    int border_width_nodes = 1;
    int n_nodes;
    if (programConfig.contains("nodes")) {
        n_nodes = programConfig["nodes"].size();
    } else {
        n_nodes = 0;
    }
    std::vector<Node> nodelist (n_nodes);
    std::vector<std::vector<int>> connections (n_nodes);
    int start_node_id = 0;
    int goal_node_id = 0;

    // Request relevant part of the config file
    if (programConfig.contains("World")){
        if (programConfig["World"].contains("OccupancyGridMap")){
            if (programConfig["World"]["OccupancyGridMap"].contains("resolution")){
                resolution = programConfig["World"]["OccupancyGridMap"]["resolution"];
            }
            if (programConfig["World"]["OccupancyGridMap"].contains("pixels_per_node")){
                pixels_per_node = (double) programConfig["World"]["OccupancyGridMap"]["pixels_per_node"];
            }
            if (programConfig["World"]["OccupancyGridMap"].contains("max_nodes_x")){
                max_nodes_x = programConfig["World"]["OccupancyGridMap"]["max_nodes_x"];
            }
            if (programConfig["World"]["OccupancyGridMap"].contains("max_nodes_y")){
                max_nodes_y = programConfig["World"]["OccupancyGridMap"]["max_nodes_y"];
            }
            if (programConfig["World"]["OccupancyGridMap"].contains("border_width_nodes")){
                border_width_nodes = programConfig["World"]["OccupancyGridMap"]["border_width_nodes"];
            }
        }
    }

    double node_resolution = resolution*pixels_per_node;

    if (programConfig.contains("nodes") && programConfig.contains("entrance") && programConfig.contains("finish") && programConfig.contains("connections"))
    {
        for(int i_node = 0; i_node < n_nodes; i_node++){
            Node new_node;
            new_node.x = node_resolution*(border_width_nodes + (double) programConfig["nodes"][i_node][0] - 0.5);
            new_node.y = node_resolution*(border_width_nodes + (double) programConfig["nodes"][i_node][1] - 0.5);
            new_node.g = INFINITY;
            new_node.h = 0.0;
            new_node.f = new_node.g + new_node.h;
            nodelist[i_node] = new_node;
            int n_conn = programConfig["connections"][i_node].size();
            for (int i_conn = 0; i_conn < n_conn; i_conn++){
                connections[i_node].push_back(programConfig["connections"][i_node][i_conn]);
            }
        }
        start_node_id = programConfig["entrance"];
        goal_node_id = programConfig["finish"];
    }
    else
    {
        std::cout << "No correct maze configuration set" << std::endl;
    }

    Planner planner;
    planner.setNodeList(nodelist);
    planner.setConnections(connections);
    planner.setStart(start_node_id);
    planner.setGoal(goal_node_id);
    planner.setPathFound(false);
    
    return planner;
}

Planner constructPlannerFromPRM(coordinates t3, PRM prm)
{
    // Default config values, will be overwritten if the config files contains the correct information
    double x_start = 0.0;
    double y_start = 0.0;
    double x_goal = 0.0;
    double y_goal = 0.0;

    // Request relevant part of the config file
    // if (programConfig.contains("start")){
    //     x_start = programConfig["start"][0];
    //     y_start = programConfig["start"][1];
    // }
    // if (programConfig.contains("goal")){
    //     x_goal = programConfig["goal"][0];
    //     y_goal = programConfig["goal"][1];
    // }

    printf("Loaded start and goal settings\n");
    
    /* Copy nodes and connections from PRM */
    int n_nodes_PRM = prm._graph.vertices.size();
    int n_edges_PRM = prm._graph.edges.size();
    int n_nodes = n_nodes_PRM + 2; // + 2 because of start and goal nodes that will be added
    std::vector<Node> nodelist (n_nodes);
    std::vector<std::vector<int>> connections (n_nodes);
    Node new_node;
    // Fill with nodes and connections from PRM (use IDs 2 and higher, 0 and 1 will be for the start and goal)
    for(int i_node = 0; i_node < n_nodes_PRM; i_node++){
        new_node.x = prm._graph.vertices[i_node].first;
        new_node.y = prm._graph.vertices[i_node].second;
        new_node.g = INFINITY;
        new_node.h = 0.0;
        new_node.f = new_node.g + new_node.h;
        nodelist[i_node + 2] = new_node;
        for (int i_edge = 0; i_edge < n_edges_PRM; i_edge++) {
            if (prm._graph.edges[i_edge].first == i_node) {
                connections[i_node + 2].push_back(prm._graph.edges[i_edge].second + 2);
            } else if (prm._graph.edges[i_edge].second == i_node) {
                connections[i_node + 2].push_back(prm._graph.edges[i_edge].first + 2);
            }
        }
    }

    printf("Loaded nodes and connections from PRM\n");

    /* Add start and goal node and connect them to nodes that were already in the PRM */
    int start_node_id = 0;
    int goal_node_id = 1;

    // Add start and goal node to nodelist and their connections (to the other nodes) to the connections list
    Node start_node;
    start_node.x = x_start;
    start_node.y = y_start;
    start_node.g = INFINITY;
    start_node.h = 0.0;
    start_node.f = start_node.g + start_node.h;
    nodelist[start_node_id] = start_node;
    std::pair<double, double> start_vert = std::pair<double, double> (start_node.x, start_node.y);
    // Create connections between start and existing vertices
    for (int j=0; j < n_nodes_PRM; j++){
        if (prm.checkIfEdgeIsValid(prm._graph.vertices[j], start_vert, prm._map, prm._resolution))
        {
            connections[start_node_id].push_back(j+2);
            connections[j+2].push_back(start_node_id);
        }
    }
    
    Node goal_node;
    goal_node.x = x_goal;
    goal_node.y = y_goal;
    goal_node.g = INFINITY;
    goal_node.h = 0.0;
    goal_node.f = goal_node.g + goal_node.h;
    nodelist[goal_node_id] = goal_node;
    std::pair<double, double> goal_vert = std::pair<double, double> (goal_node.x, goal_node.y);
    // Create connections between goal and existing vertices
    for (int j=0; j < n_nodes_PRM; j++){
        if (prm.checkIfEdgeIsValid(prm._graph.vertices[j], goal_vert, prm._map, prm._resolution)){
            connections[goal_node_id].push_back(j+2);
            connections[j+2].push_back(goal_node_id);
        }
    }

    printf("Added start and goal node\n");

    Planner planner;
    planner.setNodeList(nodelist);
    planner.setConnections(connections);
    planner.setStart(start_node_id);
    planner.setGoal(goal_node_id);
    planner.setPathFound(false);

    printf("Constructed planner!\n");
    
    return planner;
}
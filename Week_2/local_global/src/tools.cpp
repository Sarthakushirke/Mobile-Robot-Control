#include "tools.h"

#include <math.h>

nlohmann::json load_config_file(int argc, char *argv[])
{
    std::string filename;
    if (argc == 1) // No additional arguments provided. Use the default
    {
        std::cout << "Using default config-file: ";
        filename = "../config/params_maze.json";
        std::cout << filename << std::endl;
    }
    else if (argc > 1) // Use the provided config-file
    {
        std::cout << "Using User-provided config-file: ";
        filename = argv[1];
        std::cout << filename << std::endl;
    }
    // Parse the config-file which is in json format
    nlohmann::json programConfig;
    try
    {
        std::ifstream jsonFile(filename);
        programConfig = nlohmann::json::parse(jsonFile);
    }
    catch (nlohmann::detail::parse_error &e)
    {
        std::cout << "\n";
        if (e.byte > 1) // Error not at the start of the file
        {
            std::cout << "Error while parsing JSON-file. Are you providing a valid file?" << std::endl;
        }
        else
        {
            std::cout << "Error while parsing JSON-file. Does this file exsist, or is it empty?" << std::endl;
            std::cout << "Are your sure the JSON-file path is specified relative to this terminal window?" << std::endl;
            std::cout << "\n";
        }
        std::cout << "Refer to the error below to find your issue: " << std::endl;
        std::cout << e.what() << std::endl;
    }

    return programConfig;
}

Planner constructPlanner(const nlohmann::json &programConfig)
{
    // ------
    // Initialize the Planner
    // ------

    // Default config values, will be overwritten if the config files contains the correct information
    double resolution = 0.5;
    double pixels_per_node = 2;
    double node_resolution = resolution*pixels_per_node;
    int max_nodes_x = 10;
    int max_nodes_y = 10;
    int n_nodes;
    if (programConfig.contains("nodes")) {
        n_nodes = programConfig["nodes"].size();
    } else {
        n_nodes = 0;
    }
    std::vector<Node> nodelist (n_nodes);
    std::vector<std::vector<int>> connections (n_nodes);
    int entrance_node_id = 0;
    int finish_node_id = 0;

    // Request relevant part of the config file
    if (programConfig.contains("World")){
        if (programConfig["World"].contains("OccupancyGridMap")){
            if (programConfig["World"]["OccupancyGridMap"].contains("resolution") && programConfig["World"]["OccupancyGridMap"].contains("pixels_per_node") && programConfig["World"]["OccupancyGridMap"].contains("max_nodes_x") && programConfig["World"]["OccupancyGridMap"].contains("max_nodes_y")){
                max_nodes_x = programConfig["World"]["OccupancyGridMap"]["max_nodes_x"];
                max_nodes_y = programConfig["World"]["OccupancyGridMap"]["max_nodes_y"];
                resolution = programConfig["World"]["OccupancyGridMap"]["resolution"];
                pixels_per_node = (double) programConfig["World"]["OccupancyGridMap"]["pixels_per_node"];
                node_resolution = resolution*pixels_per_node;
            }
        }
    }
    if (programConfig.contains("nodes") && programConfig.contains("entrance") && programConfig.contains("finish") && programConfig.contains("connections"))
    {
        for(int i_node = 0; i_node < n_nodes; i_node++){
            Node new_node;
            new_node.x = node_resolution*(0.5*max_nodes_x - (((double) programConfig["nodes"][i_node][0]) - 0.5));
            new_node.y = node_resolution*(0.5*max_nodes_y - (((double) programConfig["nodes"][i_node][1]) - 0.5));
            new_node.g = INFINITY;
            new_node.h = 0.0;
            new_node.f = new_node.g + new_node.h;
            nodelist[i_node] = new_node;
            int n_conn = programConfig["connections"][i_node].size();
            for (int i_conn = 0; i_conn < n_conn; i_conn++){
                connections[i_node].push_back(programConfig["connections"][i_node][i_conn]);
            }
        }
        entrance_node_id = programConfig["entrance"];
        finish_node_id = programConfig["finish"];
    }
    else
    {
        ROS_WARN_STREAM("No correct maze configuration set");
    }

    Planner planner;
    planner.setNodeResolution(node_resolution);
    planner.setNodeList(nodelist);
    planner.setConnections(connections);
    planner.setEntrance(entrance_node_id);
    planner.setFinish(finish_node_id);
    planner.setPathFound(false);
    
    return planner;
}

void reportStatistics(const int &timestep, const time_point &startOfIteration, Planner *pPlanner)
{
    time_point end = std::chrono::steady_clock::now();
    std::cout << "Timestep " << timestep << " Complete. Time Taken: " << std::chrono::duration_cast<std::chrono::microseconds>(end - startOfIteration).count() / 1000.0 << " milliseconds"
              << "\n";

    std::cout << std::endl;
}

double wrapToPi(double angle)
{
    if (angle > M_PI)
    {
        int k = std::ceil(angle / 2 * M_PI);
        angle -= 2 * k * M_PI;
        // Rerun to ensure we're indeed correct
        angle = wrapToPi(angle);
    }
    else if (angle < -M_PI)
    {
        int k = std::floor(angle / 2 * M_PI);
        angle += 2 * M_PI;
        // Rerun to ensure we're indeed correct
        angle = wrapToPi(angle);
    }
    return angle;
}

void simulate_path_following(std::vector<Node> &nodelist, std::list<int> &path_node_IDs)
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

    /* Initialization */
    emc::IO emc_io;
    emc::OdometryData odom;
    emc_io.readOdometryData(odom);
    ros::Rate r(cycle_freq);
    ros::Rate r_init_waiting(init_waiting_rate);

    // Convert path node IDs to coordinates
    std::vector<std::vector<double>> path_coordinates;
    for (int i_node : path_node_IDs){
        path_coordinates.push_back({nodelist[i_node].x,nodelist[i_node].y});
    }
    int n_points = path_coordinates.size();
    double x_next, y_next, err_x, err_y, err_o;

    /* Wait until the path can be sent (when the map has been loaded) */
    r_init_waiting.sleep();
    while(!emc_io.sendPath(path_coordinates, draw_path_color, draw_path_width) && ros::ok()){
        r_init_waiting.sleep();
    }

    /* Move to the locations specified in the path one-by-one */
    for (int i=0; i<n_points; i++){
        x_next = path_coordinates[i][0];
        y_next = path_coordinates[i][1];
        err_x = odom.x - x_next;
        err_y = odom.y - y_next;
        err_o = wrapToPi(odom.a - atan2(y_next-odom.y, x_next - odom.x));
        while(err_x*err_x + err_y*err_y >= distance_threshold*distance_threshold){
            /* Read odometery data and update pose error calculations */
            emc_io.readOdometryData(odom);
            err_x = odom.x - x_next;
            err_y = odom.y - y_next;
            err_o = wrapToPi(odom.a - atan2(y_next-odom.y, x_next - odom.x));

            if(err_o >= err_o_threshold || err_o <= -err_o_threshold){
                /* Orient towards next point */
                emc_io.sendBaseReference(0.0,0.0,-vel_rotate*err_o/abs(err_o));
            } else {
                /* Move forward and keep orientation correct */
                emc_io.sendBaseReference(vel_translate,0.0,-gain_rotate_while_translate*err_o);
            }
            /* Plot path */
            emc_io.sendPath(path_coordinates, draw_path_color, draw_path_width);
            r.sleep();
        }
    }
    // Stop the movement of the robot
    emc_io.sendBaseReference(0.0,0.0,0.0);
}
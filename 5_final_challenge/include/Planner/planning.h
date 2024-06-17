#pragma once

#include <emc/io.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <list>
#include <algorithm>

#include "tools.h"
#include "../include/PRM/prm.h"


#define NODE_ID_NOT_SET -1

class Node
{
    public:
    double x; // x-coordinate [m]
    double y; // y-coordinate [m]
    double g; // cost-to-come from the start to this node
    double h; // heuristic cost-to-go from this node to the goal
    double f; // total cost (cost-to-come g + heuristic cost-to-go h)
    int parent_node_ID; // ID of the node from which node you arrived at this node with the lowest cost-to-come
};

class Planner
{
    public:

        /// @brief plan the optimal path from start to goal
        void planPath();

        /// @brief return the optimal path from start to goal in a vector of {x,y} coordinates
        std::vector<std::vector<double>> getPath();

        /// @brief Set the node list
        /// @param nodelist list of nodes with their coordinates
        void setNodeList(const std::vector<Node> &nodelist);

        /// @brief Set the start node
        /// @param start_nodeID ID of the start node
        void setStart(const int &start_nodeID);

        /// @brief Set the goal node
        /// @param goal_nodeID ID of the goal node
        void setGoal(const int &goal_nodeID);

        /// @brief Set connection list
        /// @param connections list of connections
        void setConnections(const std::vector<std::vector<int>> &connections);
        
        /// @brief Set path found boolean
        /// @param path_found boolean to indicate if the path has been found
        void setPathFound(const bool &path_found);

        /// @brief Set path node IDs
        /// @param path_node_IDs vector containing the path node IDs from start to goal
        void setPathNodeIDs(const std::list<int> &path_node_IDs);

        /// @brief list of all nodes
        std::vector<Node> _nodelist;

        /// @brief ID of the start node
        int _start_nodeID;

        /// @brief ID of the goal node
        int _goal_nodeID;

        /// @brief list of all connections per node: _connections[i] gives the indices of the nodes connected to node i (_nodelist[i])
        std::vector<std::vector<int>> _connections;

        /// @brief boolean to indicate if the path has been found
        bool _path_found;

        /// @brief list of all connections
        std::list<int> _path_node_IDs;
    
    private:

};

/// @brief Calculate the distance between two nodes
/// @param node_A the first node
/// @param node_B the second node
/// @return
double calculate_distance(Node node_A, Node node_B);

/// @brief Construct and configure the planner based on the gridmap representation
/// @param programConfig the parsed json-config file
/// @return
Planner constructPlannerFromGridmap(const nlohmann::json &programConfig);

/// @brief Construct and configure the planner using the nodes and connections from the PRM
/// @param programConfig the parsed json-config file
/// @param prm The PRM
/// @return
Planner constructPlannerFromPRM(const nlohmann::json &programConfig, PRM prm);

struct coordinates
{
    double x;
    double y;
    coordinates(double x_ , double y_)
    {
        x = x_;
        y = y_;
    }
};
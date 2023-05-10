#pragma once

#include <emc/io.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <list>
#include <algorithm>

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

        /// @brief plan the optimal path from entrance to finish
        void planPath();

        /// @brief Set the node to node distance
        /// @param node_resolution resolution of the node gird [m]
        void setNodeResolution(const double &node_resolution);

        /// @brief Set the node list
        /// @param nodelist list of nodes with their coordinates
        void setNodeList(const std::vector<Node> &nodelist);

        /// @brief Set the entrance node
        /// @param entrance_nodeID ID of the entrance node
        void setEntrance(const int &entrance_nodeID);

        /// @brief Set the finish node
        /// @param finish_nodeID ID of the finish node
        void setFinish(const int &finish_nodeID);

        /// @brief Set connection list
        /// @param connections list of connections
        void setConnections(const std::vector<std::vector<int>> &connections);
        
        /// @brief Set path found boolean
        /// @param path_found boolean to indicate if the path has been found
        void setPathFound(const bool &path_found);

        /// @brief Set path node IDs
        /// @param path_node_IDs vector containing the path node IDs from entrance to finish
        void setPathNodeIDs(const std::list<int> &path_node_IDs);

        /// @brief distance from node to node
        double _node_resolution;

        /// @brief list of all nodes
        std::vector<Node> _nodelist;

        /// @brief ID of the entrance node
        int _entrance_nodeID;

        /// @brief ID of the finish node
        int _finish_nodeID;

        /// @brief list of all connections per node: _connections[i] gives the indices of the nodes connected to node i (_nodelist[i])
        std::vector<std::vector<int>> _connections;

        /// @brief boolean to indicate if the path has been found
        bool _path_found;

        /// @brief list of all connections
        std::list<int> _path_node_IDs;
    
    private:

};

double calculate_distance(Node node_A, Node node_B);

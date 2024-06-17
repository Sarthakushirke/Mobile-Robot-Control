#pragma once

#include <emc/io.h>
#include <iostream>
#include <stdio.h>
#include <list>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "tools.h"

class Graph
{
    public:
    std::vector<std::pair<double, double>> vertices; // [x,y] coordinates in map
    std::vector<std::pair<int, int>> edges; // [id_1, id_2], where id_1 is the id of the start vertix and id_2 is the id of the end vertix 
    /// @brief add a vertex to the list
    void addVertex(const std::pair<double, double>& new_vertex) {
        vertices.emplace_back(new_vertex.first, new_vertex.second); // Add the new vertex at the end of the list
    }

    /// @brief add an edge to the list
    void addEdge(int id_1, int id_2) {  
        bool edgeExists = false;
        for (const auto& edge : edges) {
            if ((edge.first == id_1 && edge.second == id_2) || (edge.first == id_2 && edge.second == id_1)) {
                edgeExists = true;
                break;
            }
        }

        // Add the edge if it doesn't already exist
        if (!edgeExists) {
            edges.emplace_back(id_1, id_2);
        }
    }
};

struct Point {
    double x;
    double y;
};

class PRM
{
    public:
        /// @brief generate PRM vertices and edges based on a given map
        /// @param programConfig the parsed json-config file
        void generatePRM(const nlohmann::json &programConfig, bool show_PRM);

        /// @brief generate random vertex using the boundaries of the map
        std::pair<double, double> generateRandomVertex(double x_max, double y_max);

        /// @brief get distance between two vertices
        double distanceBetweenVertices(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2);

        /// @brief load the map
        cv::Mat loadPNG(std::string filename);
        
        /// @brief plot the map
        void plotMap(cv::Mat map);

        /// @brief Set the graph
        /// @param graph list of nodes with their coordinates
        void setGraph(const Graph &graph);

        /// @brief Set the map
        void setMap(const cv::Mat &map);

        /// @brief Set the resolution
        void setResolution(const double &resolution);

        /// @brief check if edge between vertex_1 and vertex_2 goes through obstacle
        bool checkIfEdgeCrossesObstacle(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2, cv::Mat map, double resolution);

        /// @brief check if an edge can be created
        bool checkIfEdgeIsValid(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2, cv::Mat map, double resolution);
        
        /// @brief inflate the walls
        cv::Mat inflateWalls(cv::Mat map, double resolution);

        /// @brief check if new vertex is not too close to existing vertices
        bool checkIfVertexIsValid(const Graph G, const std::pair<double, double>& new_vertex);
        
        /// @brief check if two vertices are within certain distance of each other
        bool checkIfVerticesAreClose(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2);
        
        /// @brief scale the PRM, show vertices and edges and save it if desired
        void scaleAndSavePRM(const Graph G, cv::Mat map, double resolution, bool show_PRM);

        Graph _graph;
        cv::Mat _map;
        double _resolution;
};
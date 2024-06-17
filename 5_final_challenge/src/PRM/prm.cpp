#include "../../include/PRM/prm.h"
#include <iostream>
#include <random>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include "./3rdparty/json.hpp"

/* Not necessary to change this value for the given maps */
#define thresh_occ      130  // Threshold for occupied/free in PNG 
#define ROWS 4
#define COLS 4

/* Code for saving necessary stuff for planner*/
void PRM::setGraph(const Graph &graph){
    _graph = graph;
}

void PRM::setMap(const cv::Mat &map){
    _map = map;
}

void PRM::setResolution(const double &resolution){
    _resolution = resolution;
}

/* Utils for map*/
cv::Mat PRM::loadPNG(std::string filename){
    return cv::imread(filename, cv::IMREAD_GRAYSCALE);
};

void PRM::scaleAndSavePRM(const Graph G, cv::Mat map, double resolution, bool show_PRM){
    // Resize the image for drawing
    double scaling_factor = ceil(0.5/resolution);
    cv::Mat scaledImage;
    cv::resize(map, scaledImage, cv::Size(scaling_factor*map.cols, scaling_factor*map.rows),0,0,cv::INTER_NEAREST);

    // Add edges to map
    for (const auto& edge : G.edges){
        int id1 = edge.first;
        int id2 = edge.second;
        cv::Point pt1(scaling_factor*G.vertices[id1].first/resolution, scaling_factor*(map.rows-G.vertices[id1].second/resolution)-1);
        cv::Point pt2(scaling_factor*G.vertices[id2].first/resolution, scaling_factor*(map.rows-G.vertices[id2].second/resolution)-1);
        cv::line(scaledImage, pt1, pt2, cv::Scalar(200, 200,200), 1); 
    }
    
    // Add vertices to map
    for (const auto& vertex : G.vertices) {       
        double x = scaling_factor*static_cast<double>(vertex.first / resolution);
        double y = scaling_factor*static_cast<double>(map.rows-vertex.second / resolution)-1;
        cv::circle(scaledImage, cv::Point(x, y), scaling_factor*0.5, cv::Scalar(127,127,127), cv::FILLED);
    } 


    if (show_PRM) {
        // Resize the image to a smaller size for display if necessary
        cv::Mat smallerImage;
        cv::resize(scaledImage, smallerImage, cv::Size(1280, 720)); // Resize to 1280x720

        // Display the smaller image in a window with specified size
        cv::namedWindow("Map", cv::WINDOW_NORMAL);
        cv::resizeWindow("Map", 1920, 1080);
        cv::imshow("Map", smallerImage);
        printf("Press button to continue code...");
        cv::waitKey(0);
    }

    // Save the map to PNG
    printf("Generated PRM is written to Generated_PRM/PRM.png\n");
    cv::imwrite("../Generated_PRM/PRM.png", scaledImage);
}

/* Utils for vertices */
std::pair<double, double> PRM::generateRandomVertex(double x_max, double y_max){
    struct timeval t;
    gettimeofday(&t, NULL);
    srand(t.tv_usec + t.tv_sec*1000000);
    double rand_x = ((double) rand() / RAND_MAX) * x_max;
    double rand_y = ((double) rand() / RAND_MAX) * y_max;
    return std::pair<double, double> (rand_x, rand_y);
}

double PRM::distanceBetweenVertices(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2) {
    double dx = vertex_1.first - vertex_2.first;
    double dy = vertex_1.second - vertex_2.second;
    return std::sqrt(dx * dx + dy * dy);
};

cv::Mat PRM::inflateWalls(cv::Mat map, double resolution){
    cv::Mat inflatedMap = map.clone();
    //-- Exercise PRM 1/3: write the necessary code to inflate the walls to account for the robot size --//
    // Iterate through each cell in the matrix
    for (int i = 0; i < inflatedMap.rows; ++i) {
        for (int j = 0; j < inflatedMap.cols; ++j) {
            // If the current cell is a wall (0) inflate it
            if (map.at<u_char>(i,j) <= 5) 
            {
                // Set the surrounding cells to 0 (wall) if they are within bounds
                for (int x = i - ROWS; x <= i + ROWS; ++x) {
                    for (int y = j - COLS; y <= j + COLS; ++y) {
                        if (x >= 0 && x < inflatedMap.rows && y >= 0 && y < inflatedMap.cols) {
                            inflatedMap.at<u_char>(x,y) = 0;
                        }
                    }
                }
            }
        }
    }

    cv::imwrite("inflatedMap.png", inflatedMap);

    //-- End Exercise PRM 1/3 --//
    return inflatedMap;
}

bool PRM::checkIfVertexIsValid(const Graph G, const std::pair<double, double>& new_vertex){
    bool acceptVert = true;
    /*-- Exercise PRM 2/3: write the necessary code to check if the new_vertex is a certain distance away from the existing vertices --*/
    // Put your code here!
    double certain_distance = 0.6;
    for (const auto& vertex : G.vertices) {
        if (distanceBetweenVertices(vertex, new_vertex) < certain_distance) {
            acceptVert = false;
            std::cout<<"Failed distance between vertices is " <<distanceBetweenVertices(vertex, new_vertex) << std::endl;
            break;
        }
        else 
            acceptVert = true;
    }
    std::cout<<"New vertex is at: " << new_vertex.first << ", " <<new_vertex.second << std::endl;
    // printf("Your boolean variable is: %s", acceptVert ? "true" : "false");
    /*-- End Exercise PRM 2/3 --*/
    return acceptVert;
}

bool PRM::checkIfEdgeIsValid(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2, cv::Mat map, double resolution){
    // Check the distance
    bool verticesAreClose = checkIfVerticesAreClose(vertex_1, vertex_2);

    // Check if edge goes through obstacle
    bool edgeCrossesObstacle = checkIfEdgeCrossesObstacle(vertex_1, vertex_2, map, resolution);

    return (verticesAreClose && !edgeCrossesObstacle);
}

bool PRM::checkIfVerticesAreClose(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2){
    bool verticesAreClose = true;
    /*-- Exercise PRM 3a/3: write the necessary code to check if a edge between two vertices is valid, 
                            i.e. does not cross an obstacle and is within certain distance from other vertices --*/
    // Put your code here!

    int certain_distance = 2;
    // for (const auto& vertex : G.vertices) {
    if (distanceBetweenVertices(vertex_1, vertex_2) > certain_distance) {
        verticesAreClose = false;
    }
    // }

    /*-- End Exercise PRM 3a/3 --*/
    return verticesAreClose;
}
bool PRM::checkIfEdgeCrossesObstacle(const std::pair<double, double>& vertex_1, const std::pair<double, double>& vertex_2, cv::Mat map, double resolution) {
    bool EdgeCrossesObstacle = false;
    /*-- Exercise PRM 3b/3: write the necessary code to check if a edge between two vertices is valid, 
                            i.e. does not cross an obstacle and is within certain distance from other vertices --*/

    cv::Mat secondMap = map.clone();
    // Convert world coordinates to image coordinates
    int x1 = static_cast<int>(vertex_1.first / resolution);
    int y1 = map.rows - 1 - static_cast<int>(vertex_1.second / resolution);
    int x2 = static_cast<int>(vertex_2.first / resolution);
    int y2 = map.rows - 1 - static_cast<int>(vertex_2.second / resolution);

    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        // Ensure x and y are within the map bounds
        if (x1 >= 0 && x1 < map.cols && y1 >= 0 && y1 < map.rows) {
            // Check if the current point intersects any black pixels (obstacles)
            if (map.at<uchar>(y1, x1) <= 128) {
                EdgeCrossesObstacle = true;
                // map.at<uchar>(y1,x1) = 140;
                break;
            }
        } else {
            // If out of bounds, we consider it intersecting an obstacle
            EdgeCrossesObstacle = true;
            // map.at<uchar>(y1,x1) = 140;
            break;
        }

        // Reached the end of the line segment
        if (x1 == x2 && y1 == y2) {
            break;
        }

        int e2 = 2*err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
        // map.at<uchar>(y1,x1) = 140;
    }
    cv::imwrite("secondMap.png", secondMap);

    /*-- End Exercise PRM 3b/3 --*/
    return EdgeCrossesObstacle;
}

/* Main code */
void PRM::generatePRM(const nlohmann::json &programConfig, bool show_PRM){
    // Initialization
    Graph G;
    
    // Iterator
    int i = 0; 

    // Number of vertices
    int N = 50;

    // Set default values
    double resolution = 0.5; // default value [meter per pixel]
    std::string filename = "../maps/random_map_1.png"; // default map

    // Request relevant part of the config file
    if (programConfig.contains("resolution")){
        resolution = programConfig["resolution"];
    }
    // Request relevant part of the config file
    if (programConfig.contains("filename")){
        filename = (std::string) programConfig["filename"];
    }

    // Load map
    cv::Mat map = loadPNG(filename);
    
    /* Exercise PRM 1/3: write the necessary code in function *inflateWalls* to inflate the walls to account for the robot size */
    cv::Mat inflatedMap = inflateWalls(map,resolution);

    // Replace map with inflated walls version
    map = inflatedMap.clone();

    // While loop to create of PRM vertices and edges
    while (i < N){
        // Get random vertex in the map
        std::pair<double, double> new_vertex = generateRandomVertex(resolution*map.cols, resolution*map.rows);
        
        // If the new_vertex is in a inflated wall, continue to next iteration
        if ( static_cast<int>( map.at<uchar>(map.rows-floor(new_vertex.second/resolution)-1, floor(new_vertex.first/resolution))) < thresh_occ) {
            continue;
        }
        
        // Check if the new vertex is too close to existing vertices
        /* Exercise PRM 2/3: write the necessary code in function *checkIfVertexIsValid* to check if the new_vertex is a certain distance away from the existing vertices */
        bool acceptVert =  checkIfVertexIsValid(G, new_vertex);

        if (!acceptVert){ // If vertex is invalid, continue to next iteration
            continue;
        }

        // Add vertex to list
        G.addVertex(new_vertex);

        // Create edges between new_vertex and existing vertices
        /* Exercise PRM 3/3: write the necessary code in function *checkIfEdgeIsValid* to check if a edge between two vertices is valid, 
                             i.e. does not cross an obstacle and is within certain distance from other vertices */
        for (int j=0; j < i; j++){            
            if (checkIfEdgeIsValid(G.vertices[j], new_vertex, map, resolution)){
                G.addEdge(i, j);
            }  
        }
        
        // Update iterator, print the number of vertices added
        i++; 
        printf("\rNumber of vertices added: %d/%d", i, N);
        fflush(stdout); 
    }

    printf("\nNumber of edges: %lu \n", G.edges.size());

    // Save graph, map and resolution for future use in the 'planner'
    setGraph(G);
    setMap(map);
    setResolution(resolution);
    
    // Scale the map for visualization purposes, add vertices and edges, show if desired, and save it to PNG 
    scaleAndSavePRM(G, map, resolution, show_PRM);
}
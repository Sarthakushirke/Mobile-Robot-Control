#pragma once

// #include "./Objects/World.h"
#include "./Objects/Robot.h"

#include "./Planner/planning.h"

#include <ros/ros.h>

#include "./3rdparty/json.hpp"

#include <iostream>
#include <fstream>

#include <chrono>

typedef std::chrono::steady_clock::time_point time_point;

/// @brief Load and parse the json file
/// @param argc
/// @param argv
/// @return
nlohmann::json load_config_file(int argc, char *argv[]);

/// @brief Construct and configure the world model
/// @param programConfig the parsed json-config file
/// @return
// World constructWorldModel(const nlohmann::json &programConfig);

/// @brief Construct and configure the planner
/// @param programConfig the parsed json-config file
/// @return
Planner constructPlanner(const nlohmann::json &programConfig);

/// @brief Log key statistics of the particle filter to the terminal
/// @param timestep The timestep index
/// @param startOfIteration The time point at which the current iteration was started
/// @param pFilt The current state of the particle filter
void reportStatistics(const int &timestep, const time_point &startOfIteration, Planner *pPlanner);

/// @brief Wraps an angle to the range [-Pi, Pi]
/// @param angle An angle in the range [-inf, inf]
/// @return An angle in the range [-pi, pi]
double wrapToPi(double angle);

/// @brief Simulate the robot following the planned path using a simple controller
/// @param nodelist Vector containing all nodes
/// @param path_node_IDs List of nodeIDs of all nodes in the path from entrance to finish
void simulate_path_following(std::vector<Node> &nodelist, std::list<int> &path_node_IDs);

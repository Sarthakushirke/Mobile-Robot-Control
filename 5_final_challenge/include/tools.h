#pragma once

#include "./Objects/Robot.h"

#include "./3rdparty/json.hpp"
#include <iostream>
#include <fstream>
#include <chrono>

typedef std::chrono::steady_clock::time_point time_point;

/// @brief Load and parse the json file (assume first argument is the filename)
/// @param argc
/// @param argv
/// @return
nlohmann::json load_config_file(int argc, char *argv[]);

/// @brief Load and parse the json file
/// @param filename
/// @return
nlohmann::json load_config_file(std::string filename);

/// @brief Construct and configure the world model
/// @param programConfig the parsed json-config file
/// @return
// World constructWorldModel(const nlohmann::json &programConfig);

/// @brief Log key statistics to the terminal
/// @param timestep The timestep index
/// @param startOfIteration The time point at which the current iteration was started
void reportStatistics(const int &timestep, const time_point &startOfIteration);

/// @brief Wraps an angle to the range [-Pi, Pi]
/// @param angle An angle in the range [-inf, inf]
/// @return An angle in the range [-pi, pi]
double wrapToPi(double angle);

#include "tools.h"

#include <math.h>

nlohmann::json load_config_file(std::string filename)
{
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
    return load_config_file(filename);
}

void reportStatistics(const int &timestep, const time_point &startOfIteration)
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
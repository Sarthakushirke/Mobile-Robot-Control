#pragma once

// EMC-enviroment
#include <emc/io.h>
#include <emc/rate.h>

class Robot
{
public:
    /// @brief Constructor of the robot object
    /// @param io pointer to the emc::IO object implementing the interfacing to the robot hardware.
    Robot(emc::IO *io);

    /// @brief Check whether we have received new laser-messages
    /// @return True: Yes we have. False: No we haven't
    bool newLaserData();

    /// @brief Get the latest laser information
    /// @return the laser message
    emc::LaserData getLaserData();

    /// @brief Check whether we have received new odom-messages
    /// @return True: Yes we have. False: No we haven't
    bool newOdomData();

    /// @brief Get the last odometry information
    /// @return the odometry message
    emc::OdometryData getOdomData();

private:
    /// @brief The pointer to the io object
    emc::IO *_io;

    /// @brief The latest laser message
    emc::LaserData _laser;

    /// @brief The latest odometry message
    emc::OdometryData _odom;
};

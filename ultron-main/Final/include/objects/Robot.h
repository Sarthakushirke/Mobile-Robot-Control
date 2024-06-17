
// EMC-enviroment
#include <emc/io.h>
#include <emc/rate.h>
// #include <./eigen3/Eigen/Dense>
#include <math.h>
#include<iostream>
#include "./DataTypes.h"
#include "./tools.h"





class Arc
{
    public: 

    /// @brief Sector scanned Ids
    sector arcLength;
    
    /// @brief Limited scanned points
    std::vector<double> _scanArc;

    /// @brief scanned hit points
    std::vector<coordinates> _hitPoints;

    /// @brief scanned open points
    std::vector<sector> _openSpace;

    /// @brief trigger for open space found
    /// @brief true when found, obviously
    bool openFound; 
    
    /// @brief trigger for Object found
    /// @brief true when obbject found, obviously
    bool objFound; 

    /// @brief Scan radius
    double _radius;


};

class Robot : public Arc
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

    void visualize_arc();

    
    void createScanArc(double radius);

    void divideArc();   

    void draw_io(std::vector<std::vector<double>> path, std::array<double, 3> color, double width = 0.02, int id = 0); 

    void sendInputs(double vx, double vy, double va);

    void open_door_sim();

    void voiceCall(const std::string& str);


private:
    /// @brief The pointer to the io object
    emc::IO *_io;

    /// @brief The latest laser message
    emc::LaserData _laser;

    /// @brief The latest odometry message
    emc::OdometryData _odom;

    

    

    
};

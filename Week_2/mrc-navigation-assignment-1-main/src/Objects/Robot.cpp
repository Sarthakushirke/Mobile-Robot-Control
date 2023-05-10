#include "./Objects/Robot.h"

Robot::Robot(emc::IO *io) : _io(io)
{
    _laser = emc::LaserData();
    _odom = emc::OdometryData();
    return;
}

bool Robot::newLaserData()
{
    return _io->readLaserData(_laser);
}

bool Robot::newOdomData()
{
    return _io->readOdometryData(_odom);
}

emc::OdometryData Robot::getOdomData()
{
    return _odom;
}

emc::LaserData Robot::getLaserData()
{
    return _laser;
}
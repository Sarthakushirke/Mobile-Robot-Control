#include "../../include/objects/Robot.h"


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



void Robot::visualize_arc()
{
	//horizon visualizer -- comment during test
	double ang, arc_x, arc_y;

	std::vector<std::vector<double>> arc_path;

   	
	for(int n = arcLength.startID; n < arcLength.endID; n++)  // or 0 to hitPoints.size() for full hitpoints
	{
		ang = (double(n) - _laser.ranges.size()/2)*_laser.angle_increment;
        ang = wrapToPi(ang+_odom.a);
		
		arc_x = _odom.x + cos(ang)*_scanArc[n];
		arc_y = _odom.y + sin(ang)*_scanArc[n];
		
        coordinates arcc, mapBot(_odom.x, _odom.y);
        arcc = bot_to_map(_scanArc[n], ang, mapBot);

		arc_path.push_back({arcc.x, arcc.y});	
		
	}
	
	_io->sendPath(arc_path, {1,0,0}, 0.02, 1);

}


void Robot::createScanArc(double radius)
{
    _scanArc.clear();

    _hitPoints.clear();

    _openSpace.clear();

    _radius = radius;

    openFound = false;

    objFound = false;


    for( int i = 0; i < _laser.ranges.size(); i++ ) //Cap on laser data
    {
         
        if(_laser.ranges[i] < radius)
        {
            _scanArc.push_back(_laser.ranges[i]);
        }	
        else
        {
            _scanArc.push_back(radius);

        }
    }

    #if SIMULATOR
    visualize_arc();
    #endif

    divideArc();
}

void Robot::divideArc()
{
    // split into open acr and hit apoints
    bool flag = false; //  false -> not found open space yet;  true -> found open space
    sector tempOpen;



    for(int i = arcLength.startID; i < arcLength.endID - 1 ; ++i)
    {
        
        if(_scanArc[i] == _radius && !flag)
        {
            flag = true;
            tempOpen.startID = i; //Found the start of open space
                                

        }
        else if(flag && (_scanArc[i] != _radius || i == arcLength.endID - 2 ))
        {
            //keep counting until not open space or end of scan arc
            flag = false;
            if(_scanArc[i] != _radius)
                tempOpen.endID = i-1;
            else
            {
                tempOpen.endID = i;
                
            }
            if((tempOpen.endID - tempOpen.startID)*_laser.angle_increment*_radius > BOT_CLEARANCE/2)
            {	
                _openSpace.push_back(tempOpen);  
                
                openFound = true;
                
                //once end of open space found push it. 
            }
                
            
        
        } 

        if(_scanArc[i] != _radius)
        {
              objFound = true;
              
        }


        if(_laser.ranges[i] < _laser.range_max && _laser.ranges[i] > _laser.range_min) //Objects
        {

          
    
            double a = (double(i) - _scanArc.size()/2)*_laser.angle_increment;	
            coordinates temp;

            temp.x = cos(a + _odom.a)*_laser.ranges[i] + _odom.x;
            temp.y = sin(a + _odom.a)*_laser.ranges[i] + _odom.y;

            _hitPoints.push_back(temp);
        }
        
    }

    #if SIMULATOR
    if(_hitPoints.size()!=0)
    {
        std::vector<std::vector<double>> arc_path;

        for(int i = 0; i<_hitPoints.size();++i)
        {
            arc_path.push_back({_hitPoints[i].x, _hitPoints[i].y});
        }    
        _io->sendPath(arc_path,{0,1,1},0.02,4);
    }
    #endif
}



void Robot::draw_io(std::vector<std::vector<double>> path, std::array<double, 3> color, double width, int id )
{
    _io->sendPath(path,color,width,id);   
}


void Robot::sendInputs(double vx, double vy, double va)
{
    _io->sendBaseReference(vx,vy,va);
}


void Robot::open_door_sim()
{
    _io->sendOpendoorRequest();
}

void Robot::voiceCall(const std::string& str)
{
    _io->speak(str);
}


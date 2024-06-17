
#include "../include/tools.h"
#include "../include/DataTypes.h"





double wrapToPi(double angle)
{
	if (angle > M_PI)
	{
		int k = std::ceil(angle / 2 * M_PI);
		angle -= 2 * k * M_PI;		// Rerun to ensure we're indeed correct
		angle = wrapToPi(angle);

	}
	else if (angle < -M_PI)
	{
		int k = std::floor(angle / 2 * M_PI);
		angle += 2 * M_PI;		// Rerun to ensure we're indeed correct
		angle = wrapToPi(angle);
	}
	return angle;
}


coordinates bot_to_map(double dist, double angle, coordinates mapBot)// angle wrt map	
{
    coordinates laser;

    laser.x = mapBot.x + cos(angle)*dist;
    laser.y = mapBot.y + sin(angle)*dist;

    return laser;
} 	

coordinates sim_world_to_map(coordinates Pose)
{
    coordinates botFrame;

    botFrame.x = Pose.y - SHIFT_Y; 
    botFrame.y = -(Pose.x - SHIFT_X);

    return botFrame;
}

coordinates sim_map_to_world(coordinates Pose)
{
    coordinates worldFrame;
    worldFrame.x = -Pose.y + SHIFT_X;
    worldFrame.y = Pose.x + SHIFT_Y;

    return worldFrame;
}
#include "../include/APF.h"





double V(coordinates X, coordinates G, double k)
{
    return 0.5*k*((X.x - G.x)*(X.x - G.x) + (X.y - G.y)*(X.y - G.y)); // norm

} 

coordinates dV(coordinates X, coordinates G, double k)
{
    coordinates temp;
    temp.x = k*(X.x - G.x);
    temp.y = k*(X.y - G.y);

    return temp;
}


double Vab(coordinates X, coordinates P)
{
    return 0.5*((X.x - P.x)*(X.x - P.x) + (X.y - P.y)*(X.y - P.y));
}

double dVab(double x, double p)
{
    return x-p;
}



double S(double Vab)
{
    double k = 100;
    return k*Vab/(1+k*Vab);
}

double dS(double Vab)
{
    double k = 100;
    return k/((1+k*Vab)*(1+k*Vab));
}


std::pair<double, double> velocity_finder(coordinates Pose, double orient, coordinates goalPose, std::vector<coordinates> hitPoints)
{
    coordinates velocity;

    double k_goal = 0.2;

    int jumps=10, cPID;
   

    velocity = dV(Pose, goalPose, k_goal);

    if(hitPoints.size() != 0)
    {
        int minPoints = 2;

        
        double closestPoint = 10000;
        for(int i = 0; i<hitPoints.size(); i+=jumps)
        {
            if(closestPoint > Vab(Pose, hitPoints[i]))
            {
                cPID = i;
                closestPoint = Vab(Pose, hitPoints[i]);
            }
        }

        //io.sendPath({{Pose.x, Pose.y},{hitPoints[cPID].x, hitPoints[cPID].y}},{0, 0, 1}, 0.05);


        velocity.x = velocity.x / S(Vab(Pose, hitPoints[cPID]));
        velocity.y = velocity.y / S(Vab(Pose, hitPoints[cPID]));


        //std::cout<<"x "<<velocity.x<< " y "<<velocity.y<<std::endl;

        velocity.x = velocity.x - V(Pose, goalPose, k_goal) * dS(Vab(Pose, hitPoints[cPID])) * dVab(Pose.x, hitPoints[cPID].x) / (S(Vab(Pose, hitPoints[cPID])) * S(Vab(Pose, hitPoints[cPID])));

        velocity.y = velocity.y - V(Pose, goalPose, k_goal) * dS(Vab(Pose, hitPoints[cPID])) * dVab(Pose.y, hitPoints[cPID].y) / (S(Vab(Pose, hitPoints[cPID])) * S(Vab(Pose, hitPoints[cPID])));

       


        
    }
    
   


    double vel_tan = -cos(orient)*velocity.x - sin(orient)*velocity.y;
    double vel_nor = sin(orient)*velocity.x - cos(orient)*(velocity.y);

 
    double gain_lin_vel = 1, gain_ang_vel = 0.5;

    double lin_vel = vel_tan * gain_lin_vel;

    double lin_vel_limit = 0.1, ang_vel_limit = 1.8;

    double ang_vel = atan2(vel_nor,vel_tan) * gain_ang_vel;

    if(abs(lin_vel)>= lin_vel_limit)
        lin_vel = lin_vel_limit;

    if(abs(ang_vel)>= ang_vel_limit)
        lin_vel = ang_vel_limit;


    return std::make_pair(lin_vel, ang_vel);


}


 

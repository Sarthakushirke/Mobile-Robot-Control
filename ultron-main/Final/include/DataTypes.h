

#ifndef DATATYPES_H
#define DATATYPES_H


#define SIMULATOR 0
#define DEBUG_MODE 0
#define DRIVE_MODE 1
#define PARTICLE_FILTER 1


 #define SHIFT_X 6.5
 #define SHIFT_Y 3




#define BOT_CLEARANCE 0.3

#define REFRESH_RATE 20 
#define GRID_LENGTH 0.025

#define LOG_ODD_THRESH 5
#define LOG_ODD_OBJ 100
#define LOG_ODD_FREE -5

#define WAIT_TIME 5




struct sector
{
	int startID;
	int endID;
};

struct coordinates
{
    double x; 
    double y;
    coordinates(double x_ , double y_)
    {
        x = x_;
        y = y_;
    }
    coordinates(){}
};


struct Node 
{
    int nodeID;

    double x;
    double y;

    int inGrid = -1;  // if the node is in a grid
};

struct Edge
{
    Node nodeA;
    Node nodeB;

    double length;  

    Edge(Node a, Node b)
    {
        nodeA = a;
        nodeB = b;
    }  
    Edge(){}

};

struct RobotStates
{
    coordinates Pose;
    double x;
    double y;
    double a;

    RobotStates(coordinates p, double ang)
    {
        Pose.x =  p.x;
        x = p.x;
        Pose.y =  p.y;
        y = p.y;
        a = ang;
    }
    RobotStates(double x_, double y_, double ang)
    {
        Pose.x =  x_;
        x = x_;
        Pose.y =  y_;
        y = y_;
        a = ang;
    }
    RobotStates(){};
};


#endif
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <math.h>
#include "../APF.h"
#include "../DataTypes.h"



class Grid
{
    public:

        double gridLength = GRID_LENGTH;

        int gridID = -1;
        
        double x, y;  // grid coordinates

        double mapX, mapY;  // For simulator map change to x and y for real robot

        //Object params
        bool mapObj = false;     
        bool og_obj;    // true for object in that grid
        bool doorFlag;        // true for object in that grid
        bool objFlag = false;           
        double logOdd = 5;   // between -100 and 100


        double map_log = LOG_ODD_FREE;
        

        int tableID = -1;

        //A-star parameters
        double g = INFINITY;
        double h = 0.0;
        double f = g + h;

        int parent_node_ID;

        std::vector<int> connections;


        //Visualizer params
        std::vector<Node> Vert;
        std::vector<Edge> side;

        // Member functions 
        void create_grid(int i, int j);

        void create_sides();
        

        void transform_grid(double shiftX, double shiftY);
    

        bool check_within_grid(Node n);
       
        bool check_within_node(Node lb, Node ub);
      
        bool check_within_edges(Edge edge1, Edge edge2);
      
};

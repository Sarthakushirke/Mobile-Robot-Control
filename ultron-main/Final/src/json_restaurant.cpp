#include <iostream>
#include <fstream>
#include "json.hpp"

#include "./map_visualization.h"
#include "../include/DataTypes.h"
#include "../include/objects/Grid.h"
#include "../include/tools.h"



int n_walls, n_nodes, n_world, n_tables,n_doors;

int n_col_grid, n_row_grid;


double wall_clearance = 0.0, gridLength = GRID_LENGTH; 

//double table_clearance = 4 * gridLength;


std::vector<Node> node_list;
std::vector<Edge> world;
std::vector<Edge> doors;
std::vector<Edge> tables;
std::vector<Edge> walls;
std::vector<std::vector<Grid>> oMap;

std::vector<Node> og_node_list;
std::vector<Edge> og_world;
std::vector<Edge> og_doors;
std::vector<Edge> og_tables;
std::vector<Edge> og_walls;


std::vector<Grid> gridList;


std::vector<std::vector<Grid>> table_grids;

std::vector<int> doorID;

coordinates t0(0.9,3.95);

coordinates t1(3.85,3.6);

coordinates t2(0.85, 2.2);

coordinates t3(2.3, 0.55);

coordinates t4(4.5, 0.55);

std::vector<coordinates> w_table = {t0, t1,t2,t3,t4};


std::vector<coordinates> map_table ;




double edge_dist(Node a, Node b)
{
    return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}



void draw_particle(std::vector<int> particleIDs )
{
     std::vector<Point> pts;
     std::cout<<"Drawing particle..."<<std::endl;
     visualization vis("vector map of the hospital",-1,8,-1,5); 
     std::vector<std::pair<Node, Node>> seg;
     std::vector<std::pair<Node, Node>> tab;

    for(int i = 0; i<n_row_grid; ++i)
        for(int j = 0; j<n_col_grid; ++j)
        {   
            oMap[i][j].create_sides();
            for(int k = 0; k<particleIDs.size();++k)
            {
                if(oMap[i][j].gridID == particleIDs[k])
                pts.push_back({oMap[i][j].x, oMap[i][j].y});
            }
        }

    vis.set_points(pts); 

    vis.show_always();



}


void Makegridlist()
{
    int n_grid = 0;
    gridList.clear();
    // Open nodes counter
    for (int i= 0; i< n_row_grid; ++i)
    {
        for (int j= 0; j< n_col_grid; ++j)
        {
            if (oMap[i][j].objFlag == false)
            {
               if(oMap[i][j].logOdd <= LOG_ODD_THRESH || oMap[i][j].doorFlag )
               {
                    gridList.push_back(oMap[i][j]);
                    n_grid ++;               
               }
            }
            
        }
    }   
}    



void link_node_to_table()
{
    // Print the finish node IDs
   

    table_grids.clear();

    table_grids.resize(5);

    for(int k = 0;  k<5; ++k)
    {
        for (int i= 0; i< n_row_grid; ++i)
        {
            for (int j= 0; j< n_col_grid; ++j)
            {
                if (oMap[i][j].tableID == k)
                {
                    table_grids[k].push_back(oMap[i][j]);
                

        
                }
            }
        }
    }

}


void draw_map()
{
    std::vector<Point> pts;
    std::vector<std::pair<Node, Node>> seg;
    std::vector<std::pair<Node, Node>> tab;
    std::vector<std::pair<Node, Node>> Box1;
    std::vector<std::pair<Node, Node>> Box2;
    std::vector<std::pair<Node, Node>> tableNodes;
    std::vector<std::pair<Node, Node>> doors;

    std::cout<<"Drawing map..."<<std::endl;
    //VIsualization of created nodes
    visualization vis("vector map of the hospital",-1,8,-1,5); 
    

    for(int i = 0; i<n_row_grid; ++i)
        for(int j = 0; j<n_col_grid; ++j)
        {   
            oMap[i][j].create_sides();
            if(oMap[i][j].objFlag == false|| true )
            {    
               
               if(oMap[i][j].logOdd <= 5 || oMap[i][j].doorFlag == true )
               {
                    for(int k = 0; k<4; ++k)
                        Box1.push_back({oMap[i][j].side[k].nodeA, oMap[i][j].side[k].nodeB });
               }
                else if(oMap[i][j].logOdd >  5)
                    for(int k = 0; k<4; ++k)
                        Box2.push_back({oMap[i][j].side[k].nodeA, oMap[i][j].side[k].nodeB });
            }

 
        }
      


    for(int i = 0; i< n_walls; ++i)
    {
        seg.push_back({walls[i].nodeA,walls[i].nodeB});   
    }


    
    
    for(int i = 0; i< n_tables*4; ++i)
    {
        tab.push_back({tables[i].nodeA, tables[i].nodeB});   
    }
        


    vis.plot_lines(seg, cv::Scalar(255, 255, 255));

    vis.plot_lines(tab, cv::Scalar(125, 255, 255));

    vis.plot_lines(Box1, cv::Scalar(200, 10, 0));

    vis.plot_lines(Box2, cv::Scalar(200, 100, 200));

    // vis.plot_lines(tableNodes, cv::Scalar(100,110,0));
    
    // vis.set_points(pts); 

    vis.show_always();

   
}


void assign_gridID()
{

    int count = 0;
    for(int i = 0; i< n_row_grid; ++i)
    {
        for(int j= 0; j< n_col_grid; ++j)
        {
            oMap[i][j].gridID = -1;
            if(oMap[i][j].objFlag == false)
            {
                if(oMap[i][j].logOdd <= LOG_ODD_THRESH || oMap[i][j].doorFlag)
                    oMap[i][j].gridID = count++;

                if(oMap[i][j].doorFlag )
                {
                    doorID.push_back(oMap[i][j].gridID);
                }
            }

            // std::cout<<i<<','<<j<<' '<<oMap[i][j].gridID<<std::endl;
        }
    }
}


void create_connections()
{
    for(int i = 0; i<n_row_grid; ++i)
    {
        for(int j = 0; j<n_col_grid; ++j)
        {
            
            oMap[i][j].connections.clear();
            if(oMap[i][j].objFlag == false && (oMap[i][j].logOdd <= LOG_ODD_THRESH || oMap[i][j].doorFlag ))
            {
                                   
                if(i-1>=0 && j-1>=0)
                {    
                    if(!oMap[i-1][j-1].objFlag)
                    { 
                        if(oMap[i-1][j-1].logOdd <= LOG_ODD_THRESH || oMap[i-1][j-1].doorFlag)
                            oMap[i][j].connections.push_back(oMap[i-1][j-1].gridID);
                        
                    }   
               
                }
                if(i-1>=0)
                {
                    
                    if(!oMap[i-1][j].objFlag)
                    {
                        if(oMap[i-1][j].logOdd <= LOG_ODD_THRESH || oMap[i-1][j].doorFlag)
                            oMap[i][j].connections.push_back(oMap[i-1][j].gridID);
                      
                    }
               
                }    
                if(i-1>=0 && j+1<n_col_grid)
                {   

                    if(!oMap[i-1][j+1].objFlag)
                    {
                        if(oMap[i-1][j+1].logOdd <= LOG_ODD_THRESH || oMap[i-1][j+1].doorFlag)
                            oMap[i][j].connections.push_back(oMap[i-1][j+1].gridID);
                     
                    }
                }
                if(j-1>=0)
                {

                    if(!oMap[i][j-1].objFlag)
                    {
                        if(oMap[i][j-1].logOdd <= LOG_ODD_THRESH || oMap[i][j-1].doorFlag)
                            oMap[i][j].connections.push_back(oMap[i][j-1].gridID);
                       
                    }    
                }
                if(j+1<n_col_grid)
                {
                    
                    if(!oMap[i][j+1].objFlag)
                    {
                        if(oMap[i][j+1].logOdd <= LOG_ODD_THRESH || oMap[i][j+1].doorFlag)
                            oMap[i][j].connections.push_back(oMap[i][j+1].gridID);
                   
                    }
                }
                if(i+1<n_row_grid && j-1>=0)
                {
                    
                    if(!oMap[i+1][j-1].objFlag)
                    {
                        if(oMap[i+1][j-1].logOdd <= LOG_ODD_THRESH || oMap[i+1][j-1].doorFlag)
                            oMap[i][j].connections.push_back(oMap[i+1][j-1].gridID);
                       
                    }
                }
                if(i+1<n_row_grid)
                {
                    
                    if(!oMap[i+1][j].objFlag)    
                    {
                        if(oMap[i+1][j].logOdd <= LOG_ODD_THRESH || oMap[i+1][j].doorFlag)
                            oMap[i][j].connections.push_back(oMap[i+1][j].gridID);
                       
                    }
                }
                if(i+1<n_row_grid && j+1<n_col_grid)
                {
                    if(!oMap[i+1][j+1].objFlag)
                    {
                        if(oMap[i+1][j+1].logOdd <= LOG_ODD_THRESH || oMap[i+1][j+1].doorFlag)
                            oMap[i][j].connections.push_back(oMap[i+1][j+1].gridID);
               
                    }
                }
            }

            
        }
    }
}


void link_node_coordinates(Node& temp)
{
    
    for(int i=0; i<n_nodes  ; ++i)
    {
       
        if(temp.nodeID == node_list[i].nodeID)
        {           
            temp.x = node_list[i].x;
            temp.y = node_list[i].y;
            break;
        }
    }
}


void print_edges(std::vector<Edge> e)
{
    for(int i =0; i<e.size(); ++i)
    {
        std::cout<<e[i].nodeA.nodeID<<"@("<<e[i].nodeA.x <<','<< e[i].nodeA.y<<") -"<<e[i].nodeB.nodeID<<"@("<<e[i].nodeB.x <<','<< e[i].nodeB.y<<"): "<<e[i].length <<std::endl;
    }
}

void print_nodes(std::vector<Node> n)
{
    for(int i =0; i<n.size(); ++i)
    {
        std::cout<<n[i].nodeID<<": "<<"("<<n[i].x<<','<<n[i].y<<")"; //<<std::endl;
    }
}



void get_table_connections()
{
    
    for(int i = 0;  i<w_table.size(); ++i)
    {
        map_table.push_back(sim_world_to_map( w_table[i]));
        
    }
    

    //Table 0tables
    double space =  0.3; // in meters
    double tableSpace = 0.1;

    int space_y = ceil(space/gridLength);
    int space_x = ceil(space/gridLength);

    int space_table = ceil(tableSpace/gridLength);

    double point_2a_x = tables[2].nodeA.x ;    // 15 node x 
    double point_2a_y = tables[2].nodeA.y ;    // 15 node y

    //For table1 16 node
    double point_2b_x = tables[2].nodeB.x ;     // 16 node x 
    double point_2b_y = tables[2].nodeB.y ;    // 16 node y
   

    //ID
    int point_15_ID_x = ceil(point_2a_x/gridLength) ;
    int point_15_ID_y = ceil(point_2a_y/gridLength) ; // "Y" will stay constant

    int point_16_ID_x = ceil(point_2b_x/gridLength) ;
    int point_16_ID_y = ceil(point_2b_y/gridLength) ; // "Y" will stay constant

    //For table0 4 node
    double point_1_x = tables[1].nodeA.x ;
    double point_1_y = tables[1].nodeA.y  ;

    int point_4_ID_x = ceil(point_1_x/gridLength) ; // "X" will stay constant
    int point_4_ID_y = ceil(point_1_y/gridLength); 

    for (int i = point_16_ID_y; i < point_4_ID_y - space_y ; ++i) {
        oMap[i][point_4_ID_x + space_table].tableID = 0; 
    }

    for (int i = point_16_ID_x + space_x; i <= point_15_ID_x ; ++i) {
        oMap[point_15_ID_y - space_table][i].tableID = 0; 
    }

    // //Table 1
  
    double point_6a_x = tables[6].nodeA.x ;
    double point_6a_y = tables[6].nodeA.y ;

    //For table1 16 node
    double point_6b_x = tables[6].nodeB.x;
    double point_6b_y = tables[6].nodeB.y  ;
   
    //ID
    int point_22_ID_x = ceil(point_6a_x/gridLength);
    int point_22_ID_y = ceil(point_6a_y/gridLength); // "Y" will stay constant
 
    int point_21_ID_x = ceil(point_6b_x/gridLength);
    int point_21_ID_y = ceil(point_6b_y/gridLength); // "Y" will stay constant

    // Table1:- From 6 to 22
    //For table0 4 node
    double point_5_x = tables[5].nodeA.x ;
    double point_5_y = tables[5].nodeA.y ;
    

    int point_6_ID_x = ceil(point_5_x/gridLength); // "X" will stay constant
    int point_6_ID_y = ceil(point_5_y/gridLength); 

    double point_7_x = tables[7].nodeB.x ;
    double point_7_y = tables[7].nodeB.y  ;
    

    int point_5_ID_x = ceil(point_7_x/gridLength); // "X" will stay constant
    int point_5_ID_y = ceil(point_7_y/gridLength); 

    for (int i = point_22_ID_y ; i < point_6_ID_y - space_y; ++i) {
        oMap[i][point_6_ID_x + space_table].tableID = 1; 
    }

    for (int i = point_21_ID_x; i <= point_22_ID_x ; ++i) {
        oMap[point_21_ID_y - space_table][i].tableID = 1; 
    }

    for (int i = point_21_ID_y ; i < point_5_ID_y - space_y ; ++i) {
        oMap[i][point_5_ID_x-space_table].tableID = 1; 
    }


    //Table2
    double point_10a_x = tables[10].nodeA.x ;
    double point_10a_y = tables[10].nodeA.y ;

    //For table1 16 node
    double point_10b_x = tables[10].nodeB.x;
    double point_10b_y = tables[10].nodeB.y;
   
  
    //ID
    int point_38_ID_x = ceil(point_10a_x/gridLength);
    int point_38_ID_y = ceil(point_10a_y/gridLength); // "Y" will stay constant
 
    int point_37_ID_x = ceil(point_10b_x/gridLength);
    int point_37_ID_y = ceil(point_10b_y/gridLength); // "Y" will stay constant

    // Table1:- From 6 to 22
    //For table0 4 node
    double point_9_x = tables[9].nodeA.x ;
    double point_9_y = tables[9].nodeA.y ;
    
    int point_20_ID_x = ceil(point_9_x/gridLength); // "X" will stay constant
    int point_20_ID_y = ceil(point_9_y/gridLength); 

    double point_8_x = tables[8].nodeA.x ;
    double point_8_y = tables[8].nodeA.y ;
    
    int point_19_ID_x = ceil(point_8_x/gridLength); // "X" will stay constant
    int point_19_ID_y = ceil(point_8_y/gridLength); 

    for (int i = point_19_ID_x ; i <= point_20_ID_x  ; ++i) {
        oMap[point_19_ID_y + space_table][i].tableID = 2; 
    }

    for (int i = point_38_ID_y  ; i < point_20_ID_y ; ++i) {
        oMap[i][point_20_ID_x + space_table].tableID = i; 
    }

    for (int i = point_37_ID_x; i <= point_38_ID_x ; ++i) 
    {
        oMap[point_37_ID_y-space_table][i].tableID = 2; 
    }

    // //Table 3

    double point_13a_x = tables[13].nodeA.x;
    double point_13a_y = tables[13].nodeA.y;

    double point_13b_x = tables[13].nodeB.x;
    double point_13b_y = tables[13].nodeB.y;
   
  
    //ID
    int point_39_ID_x = ceil(point_13a_x/gridLength);
    int point_39_ID_y = ceil(point_13a_y/gridLength); // "Y" will stay constant
 
    int point_40_ID_x = ceil(point_13b_x/gridLength);
    int point_40_ID_y = ceil(point_13b_y/gridLength); // "Y" will stay constant

    double point_14_x = tables[14].nodeB.x ;
    double point_14_y = tables[14].nodeB.y ;
    
    int point_46_ID_x = ceil(point_14_x/gridLength); // "X" will stay constant
    int point_46_ID_y = ceil(point_14_y/gridLength); 

    double point_12_x = tables[12].nodeA.x ;
    double point_12_y = tables[12].nodeA.y ;
    
    int point_45_ID_x = ceil(point_12_x/gridLength); // "X" will stay constant
    int point_45_ID_y = ceil(point_12_y/gridLength); 

    for (int i = point_45_ID_y + space_x ; i < point_39_ID_y ; ++i) {
        oMap[i][point_45_ID_x - space_table].tableID = 3; 
    }


    for (int i = point_39_ID_x ; i <= point_40_ID_x ; ++i) {
        oMap[point_40_ID_y + space_table][i].tableID = 3; 
    }

    for (int i = point_46_ID_y + space_y ; i < point_40_ID_y ; ++i) {
        oMap[i][point_40_ID_x + space_table].tableID = 3; 
    }



    // //Table 4

    double point_16a_x = tables[16].nodeA.x ;
    double point_16a_y = tables[16].nodeA.y ;

    double point_16b_x = tables[16].nodeB.x;
    double point_16b_y = tables[16].nodeB.y ;
   
  
    //ID
    int point_41_ID_x = ceil(point_16a_x/gridLength);
    int point_41_ID_y = ceil(point_16a_y/gridLength); // "Y" will stay constant
 
    int point_42_ID_x = ceil(point_16b_x/gridLength);
    int point_42_ID_y = ceil(point_16b_y/gridLength); // "Y" will stay constant

    double point_19_x = tables[19].nodeA.x ;
    double point_19_y = tables[19].nodeA.y ;
    
    int point_47_ID_x = ceil(point_19_x/gridLength); // "X" will stay constant
    int point_47_ID_y = ceil(point_19_y/gridLength); 

    for (int i = point_41_ID_x  ; i < point_42_ID_x - space_x ; ++i) {
        oMap[point_41_ID_y+space_table][i].tableID = 4; 
    }

    for (int i = point_47_ID_y + space_y; i < point_41_ID_y ; ++i) {
        oMap[i][point_47_ID_x-space_table].tableID = 4; 
    }

}

void read_json()
{
    // Read the JSON file
    std::string filename;

    std::cout << "Using map config-file: ";
    filename = "../map/final.json";
    std::cout << filename << std::endl;

    
    std::ifstream inputFile(filename);

    if (inputFile.is_open()) 
    {
        std::string jsonString((std::istreambuf_iterator<char>(inputFile)),
                               std::istreambuf_iterator<char>());

        // Parse the JSON string into a RapidJSON document
        nlohmann::json document;        

        try
        {
            std::ifstream jsonFile(filename);
            document = nlohmann::json::parse(jsonFile);
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
    
         //Access the file now 

        

        if(document.contains("points"))
        {
            n_nodes = document["points"].size();
            std::cout<<"Total nodes: "<<n_nodes<<std::endl;
        }
        else
        {
            std::cout<<"Node list not found..."<<std::endl;
        }

       

        //============================================================
        //-------------------NODE----------------------------
        //===========================================================

        for(int i = 0; i<n_nodes; ++i)
        {
            if(document["points"][i].contains("x") &&
               document["points"][i].contains("y") && 
               document["points"][i].contains("_comment"))
            {
                Node new_node;

                new_node.nodeID = (double)document["points"][i]["_comment"];
                new_node.x = (double)document["points"][i]["x"] ;
                new_node.y = (double)document["points"][i]["y"] ;

                node_list.push_back(new_node);

                
            }
            else
            {
                 std::cout<<"Something wrong with points list"<<std::endl;

            }   
        }
        

        //============================================================
        //-------------------WORLD----------------------------
        //===========================================================
        if(document.contains("world"))
        {
           n_world = document["world"].size();
           for(int i = 0; i< n_world; ++i)
           {
                Edge new_world;
                Node temp;

                temp.nodeID = (double)document["world"][i][0];
                link_node_coordinates(temp);
                new_world.nodeA = temp;

                temp.nodeID = (double)document["world"][i][1];
                link_node_coordinates(temp);
                new_world.nodeB = temp;


                new_world.length = edge_dist(new_world.nodeA, new_world.nodeB);

                world.push_back(new_world);
           }

           std::cout<<"Created world "<<n_world<<std::endl;
           
        }
        else
        {
            std::cout<<"You need to enter world vertices"<<std::endl;
        }



        //============================================================
        //-----------------------DOORS----------------------------
        //===========================================================
            
        if(document.contains("doors"))
        {
           n_doors = document["doors"].size();
           
           
           std::cout<<"Got doors: "<<n_doors <<std::endl;

        }
        else
        {
            std::cout<<"No doors found..."<<std::endl;
        }


        if(document.contains("doors") )
        {
            //get cabs
            for(int i = 0; i<n_doors; ++i)
            {
                Edge new_tab;
                Node temp;

                for(int j = 0;  j<4; ++j)
                {
                    temp.nodeID = (double)document["doors"][i][j][0];
                    link_node_coordinates(temp);
                    new_tab.nodeA = temp;

                    temp.nodeID = (double)document["doors"][i][j][1];
                    link_node_coordinates(temp);
                    new_tab.nodeB = temp;

                    new_tab.length = edge_dist(new_tab.nodeA, new_tab.nodeB);

                    doors.push_back(new_tab);
                }
                

            }
           
        }



        //============================================================
        //-------------------WALL----------------------------
        //===========================================================
            
        if(document.contains("walls"))
        {
           n_walls = document["walls"].size();
           std::cout<<"Got wall node size: "<<n_walls <<std::endl;
        }
        else
        {
            std::cout<<"No walls found..."<<std::endl;
        }

        if(document.contains("walls") )
        {
            //get walls
            for(int i = 0; i<n_walls; ++i)
            {
                Edge new_wall;
                Node temp;

                temp.nodeID = (double)document["walls"][i][0];
                link_node_coordinates(temp);
                new_wall.nodeA = temp;

                temp.nodeID = (double)document["walls"][i][1];
                link_node_coordinates(temp);
                new_wall.nodeB = temp;

                new_wall.length = edge_dist(new_wall.nodeA, new_wall.nodeB);

                walls.push_back(new_wall);

            }
            
        }


        //============================================================
        //-----------------------TABLES----------------------------
        //===========================================================
            
        if(document.contains("tables"))
        {
           n_tables = document["tables"].size();
           
           
           std::cout<<"Got Tables: "<<n_tables <<std::endl;

        }
        else
        {
            std::cout<<"No Tables found..."<<std::endl;
        }


        if(document.contains("tables") )
        {
            //get cabs
            for(int i = 0; i<n_tables; ++i)
            {
                Edge new_tab;
                Node temp;

                for(int j = 0;  j<4; ++j)
                {
                    temp.nodeID = (double)document["tables"][i][j][0];
                    link_node_coordinates(temp);
                    new_tab.nodeA = temp;

                    temp.nodeID = (double)document["tables"][i][j][1];
                    link_node_coordinates(temp);
                    new_tab.nodeB = temp;

                    new_tab.length = edge_dist(new_tab.nodeA, new_tab.nodeB);

                    tables.push_back(new_tab);
                }
                

            }
            //print_edges(tables);
        }

    } 
    else 
    {
        std::cout<< "Unable to open the JSON file." << std::endl;
    }
}


void create_map()
{
     for(int j = 0; j<n_row_grid; ++j) //row Grid
    {
        for(int k = 0; k<n_col_grid; ++k) //col Grid
        {
            //Check edges of walls   
            for(int i = 0; i< n_walls; ++i)
            {   
                Node lb = og_walls[i].nodeA;
                Node ub = og_walls[i].nodeB;                               
                
                //Check if the end nodes are in Grid
                if(oMap[j][k].check_within_grid(lb) || oMap[j][k].check_within_grid(ub))
                {
                    oMap[j][k].map_log = LOG_ODD_OBJ;
                    oMap[j][k].og_obj = true;
                   
                    break;
                }
                else
                {
                    Node midNode;
                    midNode.x = (lb.x + ub.x)/2;
                    midNode.y = (lb.y + ub.y)/2;

                    //Line search for   
                    while(edge_dist(lb,ub) > 0.1)
                    {
                        
                        if(oMap[j][k].check_within_grid(midNode))
                        {
                            oMap[j][k].map_log = LOG_ODD_OBJ;
                            oMap[j][k].og_obj = true;
                            
                            break;
                        }
                        else
                        {
                            
                                    
                            if(lb.x != midNode.x && lb.y != midNode.y)
                            {
                                if(oMap[j][k].check_within_node(lb,midNode))
                                {
                                    ub = midNode;
                                }
                                else
                                {
                                    lb = midNode;
                                }
                                
                            }
                            else
                            {
                                
                               
                               if(lb.x == midNode.x)
                               {
                                    
                                    if( lb.x > oMap[j][k].x-gridLength/2 &&   lb.x < oMap[j][k].x+gridLength/2)
                                    {
                                        if((oMap[j][k].y < midNode.y && oMap[j][k].y > lb.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < lb.y))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].y < midNode.y && oMap[j][k].y > ub.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < ub.y))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                            
                                            break;
                                        }
                                    }
                                    else
                                    {
                                       
                                        break;
                                    }
                               }
                               else if(lb.y == midNode.y)
                               {
                                   
                                    if( lb.y > oMap[j][k].y-gridLength/2 &&   lb.y < oMap[j][k].y+gridLength/2)
                                    {
                                        if((oMap[j][k].x < midNode.x && oMap[j][k].x > lb.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < lb.x))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].x < midNode.x && oMap[j][k].x > ub.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < ub.x))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                           
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        
                                        break;
                                    }
                                }
                               
                                
                            }


                            midNode.x = (lb.x + ub.x)/2;
                            midNode.y = (lb.y + ub.y)/2;

                        }    
                    }   
                }                           
                
            }


            //Check inside walls
            for(int i = 0; i<n_walls/4; ++i)
            {
                if( oMap[j][k].check_within_edges(og_walls[4*i],og_walls[4*i+2]))
                {
                    
                    oMap[j][k].map_log = LOG_ODD_OBJ;
                    oMap[j][k].og_obj = true;
                }

               
            }


            //Clear outside world
            Edge e1(node_list[51], node_list[52]), e2(node_list[29],node_list[27]);

            if(oMap[j][k].check_within_edges(e1,e2))
            {
                oMap[j][k].og_obj = true;
                oMap[j][k].map_log = LOG_ODD_OBJ;

            }
            
 
            //Check inside tables
            for(int i = 0; i<n_tables; ++i)
            {
                if( oMap[j][k].check_within_edges(tables[4*i],tables[4*i+2]))
                {
                    
                    oMap[j][k].og_obj = true;
                    oMap[j][k].map_log = LOG_ODD_OBJ;
                }

               
            }

            //Check inside doors
            for(int i = 0; i<n_doors; ++i)
            {
                if( oMap[j][k].check_within_edges(doors[4*i],doors[4*i+2]))
                {
                    
                    oMap[j][k].doorFlag = true;
                    oMap[j][k].map_log = 0;
                }

               
            }

            //Check door edges

            for(int i = 0; i<n_doors*4;++i)
            {
                Node lb = doors[i].nodeA;
                Node ub = doors[i].nodeB; 
            
                
                if(oMap[j][k].check_within_grid(lb) || oMap[j][k].check_within_grid(ub))
                {
                    oMap[j][k].doorFlag = true;
                    oMap[j][k].map_log = 0;
                    
                    break;
                }
                else
                {
                    Node midNode;
                    midNode.x = (lb.x + ub.x)/2;
                    midNode.y = (lb.y + ub.y)/2;
        
                    

                    while(edge_dist(lb,ub) > 0.1)
                    {
                        
                        if(oMap[j][k].check_within_grid(midNode))
                        {
                            oMap[j][k].doorFlag = true;
                            oMap[j][k].map_log = 0;
                            
                            break;
                        }
                        else
                        {
                            
                            if(lb.x != midNode.x && lb.y != midNode.y)
                            {
                                if(oMap[j][k].check_within_node(lb,midNode))
                                {
                                    ub = midNode;
                                }
                                else
                                {
                                    lb = midNode;
                                }
                              
                            }
                            else
                            {
                                
                               
                               if(lb.x == midNode.x)
                               {
                                    
                                    if( lb.x > oMap[j][k].x-gridLength/2 &&   lb.x < oMap[j][k].x+gridLength/2)
                                    {
                                        if((oMap[j][k].y < midNode.y && oMap[j][k].y > lb.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < lb.y))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].y < midNode.y && oMap[j][k].y > ub.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < ub.y))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                          
                                            break;
                                        }
                                    }
                                    else
                                    {
                                       
                                        break;
                                    }
                               }
                               else if(lb.y == midNode.y)
                               {
                                    
                                    if( lb.y > oMap[j][k].y-gridLength/2 &&   lb.y < oMap[j][k].y+gridLength/2)
                                    {
                                        if((oMap[j][k].x < midNode.x && oMap[j][k].x > lb.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < lb.x))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].x < midNode.x && oMap[j][k].x > ub.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < ub.x))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                            
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        
                                        break;
                                    }
                                }
                               
                                
                            }


                            midNode.x = (lb.x + ub.x)/2;
                            midNode.y = (lb.y + ub.y)/2;

                        }    
                    }
                    

                }



            //Checking the edges of the Grid
            for(int i = 0; i<n_tables*4;++i)
            {
                Node lb = tables[i].nodeA;
                Node ub = tables[i].nodeB; 
            
                
                if(oMap[j][k].check_within_grid(lb) || oMap[j][k].check_within_grid(ub))
                {
                    oMap[j][k].og_obj = true;
                    oMap[j][k].map_log = LOG_ODD_OBJ;
                    
                    break;
                }
                else
                {
                    Node midNode;
                    midNode.x = (lb.x + ub.x)/2;
                    midNode.y = (lb.y + ub.y)/2;

                    

                    while(edge_dist(lb,ub) > 0.1)
                    {
                        
                        if(oMap[j][k].check_within_grid(midNode))
                        {
                            oMap[j][k].og_obj = true;
                            oMap[j][k].map_log = LOG_ODD_OBJ;
                            
                            break;
                        }
                        else
                        {
                            
                            if(lb.x != midNode.x && lb.y != midNode.y)
                            {
                                if(oMap[j][k].check_within_node(lb,midNode))
                                {
                                    ub = midNode;
                                }
                                else
                                {
                                    lb = midNode;
                                }
                              
                            }
                            else
                            {
                                
                               
                               if(lb.x == midNode.x)
                               {
                                    
                                    if( lb.x > oMap[j][k].x-gridLength/2 &&   lb.x < oMap[j][k].x+gridLength/2)
                                    {
                                        if((oMap[j][k].y < midNode.y && oMap[j][k].y > lb.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < lb.y))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].y < midNode.y && oMap[j][k].y > ub.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < ub.y))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                          
                                            break;
                                        }
                                    }
                                    else
                                    {
                                       
                                        break;
                                    }
                               }
                               else if(lb.y == midNode.y)
                               {
                                    
                                    if( lb.y > oMap[j][k].y-gridLength/2 &&   lb.y < oMap[j][k].y+gridLength/2)
                                    {
                                        if((oMap[j][k].x < midNode.x && oMap[j][k].x > lb.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < lb.x))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].x < midNode.x && oMap[j][k].x > ub.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < ub.x))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                            
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        
                                        break;
                                    }
                                }
                               
                                
                            }


                            midNode.x = (lb.x + ub.x)/2;
                            midNode.y = (lb.y + ub.y)/2;

                        }    
                    }
                    

                }
            }
            
        }
    }
    }
}

void read_original()
{
    // Read the JSON file
    std::string filename;

    std::cout << "Using map config-file: ";
    filename = "../map/og_map.json";
    std::cout << filename << std::endl;

    
    std::ifstream inputFile(filename);

    if (inputFile.is_open()) 
    {
        std::string jsonString((std::istreambuf_iterator<char>(inputFile)),
                               std::istreambuf_iterator<char>());

        // Parse the JSON string into a RapidJSON document
        nlohmann::json document;        

        try
        {
            std::ifstream jsonFile(filename);
            document = nlohmann::json::parse(jsonFile);
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
    
         //Access the file now 

        

        if(document.contains("points"))
        {
            n_nodes = document["points"].size();
            std::cout<<"Total nodes: "<<n_nodes<<std::endl;
        }
        else
        {
            std::cout<<"Node list not found..."<<std::endl;
        }

       

        //============================================================
        //-------------------NODE----------------------------
        //===========================================================

        for(int i = 0; i<n_nodes; ++i)
        {
            if(document["points"][i].contains("x") &&
               document["points"][i].contains("y") && 
               document["points"][i].contains("_comment"))
            {
                Node new_node;

                new_node.nodeID = (double)document["points"][i]["_comment"];
                new_node.x = (double)document["points"][i]["x"] ;
                new_node.y = (double)document["points"][i]["y"] ;

                og_node_list.push_back(new_node);

                
            }
            else
            {
                 std::cout<<"Something wrong with points list"<<std::endl;

            }   
        }
        

        //============================================================
        //-------------------WORLD----------------------------
        //===========================================================
        if(document.contains("world"))
        {
           n_world = document["world"].size();
           for(int i = 0; i< n_world; ++i)
           {
                Edge new_world;
                Node temp;

                temp.nodeID = (double)document["world"][i][0];
                link_node_coordinates(temp);
                new_world.nodeA = temp;

                temp.nodeID = (double)document["world"][i][1];
                link_node_coordinates(temp);
                new_world.nodeB = temp;


                new_world.length = edge_dist(new_world.nodeA, new_world.nodeB);

                og_world.push_back(new_world);
           }

           std::cout<<"Created world "<<n_world<<std::endl;
           
        }
        else
        {
            std::cout<<"You need to enter world vertices"<<std::endl;
        }



        //============================================================
        //-----------------------DOORS----------------------------
        //===========================================================
            
        if(document.contains("doors"))
        {
           n_doors = document["doors"].size();
           
           
           std::cout<<"Got doors: "<<n_doors <<std::endl;

        }
        else
        {
            std::cout<<"No doors found..."<<std::endl;
        }


        if(document.contains("doors") )
        {
            //get cabs
            for(int i = 0; i<n_doors; ++i)
            {
                Edge new_tab;
                Node temp;

                for(int j = 0;  j<4; ++j)
                {
                    temp.nodeID = (double)document["doors"][i][j][0];
                    link_node_coordinates(temp);
                    new_tab.nodeA = temp;

                    temp.nodeID = (double)document["doors"][i][j][1];
                    link_node_coordinates(temp);
                    new_tab.nodeB = temp;

                    new_tab.length = edge_dist(new_tab.nodeA, new_tab.nodeB);

                    og_doors.push_back(new_tab);
                }
                

            }
           
        }



        //============================================================
        //-------------------WALL----------------------------
        //===========================================================
            
        if(document.contains("walls"))
        {
           n_walls = document["walls"].size();
           std::cout<<"Got wall node size: "<<n_walls <<std::endl;
        }
        else
        {
            std::cout<<"No walls found..."<<std::endl;
        }

        if(document.contains("walls") )
        {
            //get walls
            for(int i = 0; i<n_walls; ++i)
            {
                Edge new_wall;
                Node temp;

                temp.nodeID = (double)document["walls"][i][0];
                link_node_coordinates(temp);
                new_wall.nodeA = temp;

                temp.nodeID = (double)document["walls"][i][1];
                link_node_coordinates(temp);
                new_wall.nodeB = temp;

                new_wall.length = edge_dist(new_wall.nodeA, new_wall.nodeB);

                og_walls.push_back(new_wall);

            }
            
        }


        //============================================================
        //-----------------------TABLES----------------------------
        //===========================================================
            
        if(document.contains("tables"))
        {
           n_tables = document["tables"].size();
           
           
           std::cout<<"Got Tables: "<<n_tables <<std::endl;

        }
        else
        {
            std::cout<<"No Tables found..."<<std::endl;
        }


        if(document.contains("tables") )
        {
            //get cabs
            for(int i = 0; i<n_tables; ++i)
            {
                Edge new_tab;
                Node temp;

                for(int j = 0;  j<4; ++j)
                {
                    temp.nodeID = (double)document["tables"][i][j][0];
                    link_node_coordinates(temp);
                    new_tab.nodeA = temp;

                    temp.nodeID = (double)document["tables"][i][j][1];
                    link_node_coordinates(temp);
                    new_tab.nodeB = temp;

                    new_tab.length = edge_dist(new_tab.nodeA, new_tab.nodeB);

                    og_tables.push_back(new_tab);
                }
                

            }
            //print_edges(tables);
        }

    } 
    else 
    {
        std::cout<< "Unable to open the JSON file." << std::endl;
    }

    void create_map();
}


void  discretize()
{

    //Discretize into oMap - find row and col count

    world[0].length = edge_dist(world[0].nodeA, world[0].nodeB);
    n_col_grid = floor((world[0].length - wall_clearance) / gridLength + 0.1);

    world[1].length = edge_dist(world[1].nodeA, world[1].nodeB);
    n_row_grid = floor((world[1].length - wall_clearance) / gridLength + 0.1);

    std::cout<<"Discretized map into: "<<n_row_grid<< " rows and "<< n_col_grid<<" cols"<<std::endl;

    //allocate memory based on discretization
    oMap.resize(n_row_grid, std::vector<Grid>(n_col_grid));

    //Finding Grid center coordinates
    for(int i = 0; i<n_row_grid; ++i)
        for(int j = 0; j< n_col_grid; ++j)
        {
            oMap[i][j].create_grid(i, j);       

            coordinates mapCoord(oMap[i][j].x,oMap[i][j].y);
            mapCoord = sim_world_to_map(mapCoord);

            oMap[i][j].mapX = mapCoord.x;
            oMap[i][j].mapY = mapCoord.y;
        }

        
    
    //Check if node is in a Grid
    
    for(int j = 0; j<n_row_grid; ++j) //row Grid
    {
        for(int k = 0; k<n_col_grid; ++k) //col Grid
        {
            //Check walls   
            for(int i = 0; i< n_walls; ++i)
            {   
                Node lb = walls[i].nodeA;
                Node ub = walls[i].nodeB;                               
                
                //Check if the end nodes are in Grid
                if(oMap[j][k].check_within_grid(lb) || oMap[j][k].check_within_grid(ub))
                {
                    oMap[j][k].mapObj = true;

                    oMap[j][k].objFlag = true;
                   
                    break;
                }
                else
                {
                    Node midNode;
                    midNode.x = (lb.x + ub.x)/2;
                    midNode.y = (lb.y + ub.y)/2;

                    //Line search for   
                    while(edge_dist(lb,ub) > 0.1)
                    {
                        
                        if(oMap[j][k].check_within_grid(midNode))
                        {
                            oMap[j][k].mapObj = true;

                            oMap[j][k].objFlag = true;
                            
                            break;
                        }
                        else
                        {
                            
                                    
                            if(lb.x != midNode.x && lb.y != midNode.y)
                            {
                                if(oMap[j][k].check_within_node(lb,midNode))
                                {
                                    ub = midNode;
                                }
                                else
                                {
                                    lb = midNode;
                                }
                                
                            }
                            else
                            {
                                
                               
                               if(lb.x == midNode.x)
                               {
                                    
                                    if( lb.x > oMap[j][k].x-gridLength/2 &&   lb.x < oMap[j][k].x+gridLength/2)
                                    {
                                        if((oMap[j][k].y < midNode.y && oMap[j][k].y > lb.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < lb.y))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].y < midNode.y && oMap[j][k].y > ub.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < ub.y))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                            
                                            break;
                                        }
                                    }
                                    else
                                    {
                                       
                                        break;
                                    }
                               }
                               else if(lb.y == midNode.y)
                               {
                                   
                                    if( lb.y > oMap[j][k].y-gridLength/2 &&   lb.y < oMap[j][k].y+gridLength/2)
                                    {
                                        if((oMap[j][k].x < midNode.x && oMap[j][k].x > lb.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < lb.x))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].x < midNode.x && oMap[j][k].x > ub.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < ub.x))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                           
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        
                                        break;
                                    }
                                }
                               
                                
                            }


                            midNode.x = (lb.x + ub.x)/2;
                            midNode.y = (lb.y + ub.y)/2;

                        }    
                    }   
                }                           
                
            }


            //Check inside walls
            for(int i = 0; i<n_walls/4; ++i)
            {
                if( oMap[j][k].check_within_edges(walls[4*i],walls[4*i+2]))
                {
                    
                    oMap[j][k].mapObj = true;

                    oMap[j][k].objFlag = true;
                }

               
            }
            
            //Outside world
            Edge e1(node_list[51], node_list[52]), e2(node_list[29],node_list[27]);

            if(oMap[j][k].check_within_edges(e1,e2))
            {
                oMap[j][k].mapObj = true;

                oMap[j][k].objFlag = true;
            }

            //Clear nodes bw wall and table
            Edge e3(node_list[53], node_list[54]), e4(node_list[19], node_list[37]);
            
            if(oMap[j][k].check_within_edges(e3,e4))
            {
                oMap[j][k].mapObj = true;

                oMap[j][k].objFlag = true;
            }
 
            //Check inside tables
            for(int i = 0; i<n_tables; ++i)
            {
                if( oMap[j][k].check_within_edges(tables[4*i],tables[4*i+2]))
                {
                    
                    oMap[j][k].mapObj = true;

                    oMap[j][k].objFlag = true;
                }

               
            }

            //Check inside doors
            for(int i = 0; i<n_doors; ++i)
            {
                if( oMap[j][k].check_within_edges(doors[4*i],doors[4*i+2]))
                {
                    
                    oMap[j][k].doorFlag = true;
                }

               
            }

            //Check door edges

            for(int i = 0; i<n_doors*4;++i)
            {
                Node lb = doors[i].nodeA;
                Node ub = doors[i].nodeB; 
            
                
                if(oMap[j][k].check_within_grid(lb) || oMap[j][k].check_within_grid(ub))
                {
                    oMap[j][k].doorFlag = true;
                    
                    break;
                }
                else
                {
                    Node midNode;
                    midNode.x = (lb.x + ub.x)/2;
                    midNode.y = (lb.y + ub.y)/2;
        
                    

                    while(edge_dist(lb,ub) > 0.1)
                    {
                        
                        if(oMap[j][k].check_within_grid(midNode))
                        {
                            oMap[j][k].doorFlag = true;
                            
                            break;
                        }
                        else
                        {
                            
                            if(lb.x != midNode.x && lb.y != midNode.y)
                            {
                                if(oMap[j][k].check_within_node(lb,midNode))
                                {
                                    ub = midNode;
                                }
                                else
                                {
                                    lb = midNode;
                                }
                              
                            }
                            else
                            {
                                
                               
                               if(lb.x == midNode.x)
                               {
                                    
                                    if( lb.x > oMap[j][k].x-gridLength/2 &&   lb.x < oMap[j][k].x+gridLength/2)
                                    {
                                        if((oMap[j][k].y < midNode.y && oMap[j][k].y > lb.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < lb.y))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].y < midNode.y && oMap[j][k].y > ub.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < ub.y))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                          
                                            break;
                                        }
                                    }
                                    else
                                    {
                                       
                                        break;
                                    }
                               }
                               else if(lb.y == midNode.y)
                               {
                                    
                                    if( lb.y > oMap[j][k].y-gridLength/2 &&   lb.y < oMap[j][k].y+gridLength/2)
                                    {
                                        if((oMap[j][k].x < midNode.x && oMap[j][k].x > lb.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < lb.x))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].x < midNode.x && oMap[j][k].x > ub.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < ub.x))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                            
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        
                                        break;
                                    }
                                }
                               
                                
                            }


                            midNode.x = (lb.x + ub.x)/2;
                            midNode.y = (lb.y + ub.y)/2;

                        }    
                    }
                    

                }



            //Checking the edges of the Grid
            for(int i = 0; i<n_tables*4;++i)
            {
                Node lb = tables[i].nodeA;
                Node ub = tables[i].nodeB; 
            
                
                if(oMap[j][k].check_within_grid(lb) || oMap[j][k].check_within_grid(ub))
                {
                    oMap[j][k].mapObj = true;

                    oMap[j][k].objFlag = true;
                    
                    break;
                }
                else
                {
                    Node midNode;
                    midNode.x = (lb.x + ub.x)/2;
                    midNode.y = (lb.y + ub.y)/2;

                    

                    while(edge_dist(lb,ub) > 0.1)
                    {
                        
                        if(oMap[j][k].check_within_grid(midNode))
                        {
                            oMap[j][k].mapObj = true;


                            oMap[j][k].objFlag = true;      
                            
                            break;
                        }
                        else
                        {
                            
                            if(lb.x != midNode.x && lb.y != midNode.y)
                            {
                                if(oMap[j][k].check_within_node(lb,midNode))
                                {
                                    ub = midNode;
                                }
                                else
                                {
                                    lb = midNode;
                                }
                              
                            }
                            else
                            {
                                
                               
                               if(lb.x == midNode.x)
                               {
                                    
                                    if( lb.x > oMap[j][k].x-gridLength/2 &&   lb.x < oMap[j][k].x+gridLength/2)
                                    {
                                        if((oMap[j][k].y < midNode.y && oMap[j][k].y > lb.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < lb.y))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].y < midNode.y && oMap[j][k].y > ub.y) || (oMap[j][k].y > midNode.y && oMap[j][k].y < ub.y))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                          
                                            break;
                                        }
                                    }
                                    else
                                    {
                                       
                                        break;
                                    }
                               }
                               else if(lb.y == midNode.y)
                               {
                                    
                                    if( lb.y > oMap[j][k].y-gridLength/2 &&   lb.y < oMap[j][k].y+gridLength/2)
                                    {
                                        if((oMap[j][k].x < midNode.x && oMap[j][k].x > lb.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < lb.x))
                                        {
                                            ub = midNode;
                                        }
                                        else if((oMap[j][k].x < midNode.x && oMap[j][k].x > ub.x) || (oMap[j][k].x > midNode.x && oMap[j][k].x < ub.x))
                                        {
                                            lb = midNode;
                                        }
                                        else 
                                        {
                                            
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        
                                        break;
                                    }
                                }
                               
                                
                            }


                            midNode.x = (lb.x + ub.x)/2;
                            midNode.y = (lb.y + ub.y)/2;

                        }    
                    }
                    

                }
            }
            
        }
    }
    }
 
    
}






#include <vector>
#include <random>
#include<iostream>
#include "../../include/DataTypes.h"
#include "../../include/objects/Grid.h"



std::vector<int> createParticle(int M)
{
    std::vector<int> selectedElements;
    
    // Seed the random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Flatten the 2D vector into a 1D vector
    std::vector<Grid> tempGrid;
    tempGrid = gridList;
   
    
    // Shuffle the flattened vector
    std::shuffle(tempGrid.begin(), tempGrid.end(), gen);
    
    // Select M random elements less than -1
    for (int i = 0; i < tempGrid.size(); ++i) {
        if (tempGrid[i].logOdd < LOG_ODD_THRESH) 
        {
            selectedElements.push_back(tempGrid[i].gridID);
            if (selectedElements.size() == M) 
            {
                break;
            }
        }
    }
    
    return selectedElements;
}



void localization()
{
    double M = 100;

    std::vector<int> initParticle_gridID = createParticle(M);

    draw_particle(initParticle_gridID);

}

    








#include "../include/tools.h"



std::vector<coordinates> Bresenham(coordinates p1, coordinates p2, double gridLength)
{
    std::vector<coordinates> indices;

    p1.x = std::ceil(p1.x/gridLength);
    p1.y = std::ceil(p1.y/gridLength);
    p2.x = std::ceil(p2.x/gridLength);
    p2.y = std::ceil(p2.y/gridLength);

    p1.y = -p1.y;
    p2.y = -p2.y;

    int dx = std::abs(p2.x - p1.x);
    int dy = std::abs(p2.y - p1.y);
    int sx = (p1.x < p2.x) ? 1 : -1;
    int sy = (p1.y < p2.y) ? 1 : -1;
    int err = dx - dy; 

    while (true) 
    {
        coordinates temp(p1.x,p1.y);
        temp.y = -temp.y;
        indices.push_back(temp);

        if (p1.x == p2.x && p1.y == p2.y) 
        {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) 
        {
            err -= dy;
            p1.x += sx;
        }
        if (e2 < dx) 
        {
            err += dx;
            p1.y += sy;
        }
    }

    return indices;
}




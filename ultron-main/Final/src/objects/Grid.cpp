#include "../../include/objects/Grid.h"



void Grid::create_grid(int i, int j)
{
    x = (j + 1) * gridLength  - gridLength / 2;
    y = (i + 1) * gridLength  - gridLength / 2;
}



bool Grid::check_within_edges(Edge edge1, Edge edge2)
{
    Node node1 = edge1.nodeA;
    Node node2 = edge1.nodeB;
    Node node3 = edge2.nodeA;
    Node node4 = edge2.nodeB;

    double minX = std::min({node1.x, node2.x, node3.x, node4.x});
    double minY = std::min({node1.y, node2.y, node3.y, node4.y});
    double maxX = std::max({node1.x, node2.x, node3.x, node4.x});
    double maxY = std::max({node1.y, node2.y, node3.y, node4.y});

    if (x >= minX && x <= maxX && y >= minY && y <= maxY)
    {
        return true;
    }

    return false;
}

 bool Grid::check_within_node(Node lb, Node ub)
{
    if (((x > lb.x && x < ub.x) && ((y < lb.y && y > ub.y) || (y > lb.y && y < ub.y))) ||
        ((x < lb.x && x > ub.x) && ((y < lb.y && y > ub.y) || (y > lb.y && y < ub.y))))
    {
        return true;
    }
    else
        return false;
}

 bool Grid::check_within_grid(Node n)
{
    if ((n.x - (x - gridLength / 2) >= 0.001 && n.x - (x + gridLength / 2) <= -0.001) &&
        (n.y - (y - gridLength / 2) >= 0.001 && n.y - (y + gridLength / 2) <= -0.001))
        return true;
    else
        return false;
}

 void Grid::create_sides()
{
    Node temp;
    temp.x = x - gridLength / 2;
    temp.y = y - gridLength / 2;
    Vert.push_back(temp);

    temp.x = x + gridLength / 2;
    temp.y = y - gridLength / 2;
    Vert.push_back(temp);

    temp.x = x + gridLength / 2;
    temp.y = y + gridLength / 2;
    Vert.push_back(temp);

    temp.x = x - gridLength / 2;
    temp.y = y + gridLength / 2;
    Vert.push_back(temp);

    Edge tempS;

    int count = 0;

    while (count <= 4)
    {
        tempS.nodeA = Vert[count];
        tempS.nodeB = Vert[(count + 1) % 4];

        side.push_back(tempS);

        count++;
    }
}

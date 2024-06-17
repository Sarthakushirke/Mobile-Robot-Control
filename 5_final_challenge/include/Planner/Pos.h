#include <math.h>

class Position{
    private:
    double a;
    double x;
    double y;
    double pi = 3.14159265359;

    public:
    void initPosition(double initX, double initY, double initA);
    void PosFromOdom(double rx, double ry, double ra);
    double getX();
    double getY();
    double getA();
};
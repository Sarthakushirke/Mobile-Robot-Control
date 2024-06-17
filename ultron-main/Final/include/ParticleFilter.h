

#include "./DataTypes.h"


class Particle
{
    private: 
        RobotStates particlePose;

        double weight;


    public: 
        double getWeight();

        RobotStates getParticlePose();

        



};
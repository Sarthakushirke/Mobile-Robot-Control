#include "../include/ParticleFilter.h"




RobotStates Particle::getParticlePose()
{
    return particlePose;
}


double Particle::getWeight()
{
    return weight;
}
#include "../include/boid.hpp"
#include "boid.hpp"

Boid::Boid(vPose pose_, int fov_, double maxSpeed_, double maxAngVelocity_)
    : pose(pose_), fov(fov_), maxSpeed(maxSpeed_), maxAngVelocity(maxAngVelocity_) {}

// Setters
void Boid::setTimeStep(int timeStep_) {
    timeStep = timeStep_;
}

void Boid::SeparationCorrection(std::vector<Boid*>)
{
    
}
void Boid::AlignmentCorrection(std::vector<Boid*>)
{
}
void Boid::CohesionCorrection(std::vector<Boid*>)
{
}

vPose Boid::getPose() const {
    return pose;
}

Boid::~Boid() {}
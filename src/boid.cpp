#include "../include/boid.hpp"

Boid::Boid(vPose pose_, int fov_, double maxSpeed_, double maxAngVelocity_)
    : pose(pose_), fov(fov_), maxSpeed(maxSpeed_), maxAngVelocity(maxAngVelocity_) {}

// Setters
void Boid::setTimeStep(int timeStep_) {
    timeStep = timeStep_;
}

Boid::~Boid() {}
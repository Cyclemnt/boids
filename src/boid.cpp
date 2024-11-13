#include "../include/boid.hpp"
#include "boid.hpp"
#include <cmath>

Boid::Boid(vPose pose_, int fov_, double speed_, double angVelocity_)
    : pose(pose_), fov(fov_), speed(speed_), angVelocity(angVelocity_) {}

// Setters
void Boid::setTimeStep(int timeStep_) {
    timeStep = timeStep_;
}

void Boid::move(int envWidth, int envHeight) {
    pose.x += speed * (timeStep / 1000) * cos(pose.theta);
    pose.y += speed * (timeStep / 1000) * sin(pose.theta);

    pose.x = fmod(pose.x, envWidth);
    pose.y = fmod(pose.y, envHeight);
}

void Boid::applyRules(Interaction interaction, std::vector<Boid*> boids) {
    vPose avgPose = {0, 0, 0};

    for (int i = 0; i < boids.size(); i++) {
        avgPose = avgPose + boids[i]->getPose();
    }
    avgPose = avgPose / boids.size();
    vPose relPose = pose - avgPose;
    
    int dir = 1;
    if (interaction == Interaction::DISTANCING) {
        dir += (fmod(atan2(-relPose.y, -relPose.x), 2*M_PI) < fmod(pose.theta, 2*M_PI)) * -2;
    }
    else if (interaction == Interaction::ALIGNMENT) {
        dir += (fmod(avgPose.theta, 2*M_PI) < fmod(pose.theta, 2*M_PI)) * -2;
    }
    else if (interaction == Interaction::COHESION) {
        dir += (fmod(atan2(relPose.y, relPose.x), 2*M_PI) < fmod(pose.theta, 2*M_PI)) * -2;
    }
    pose.theta += dir * (angVelocity * (timeStep / 1000));
    pose.theta = fmod(pose.theta, 2*M_PI);
}

vPose Boid::getPose() const {
    return pose;
}
Interaction Boid::getCurrentInteraction() const {
    return currentInteraction;
}

Boid::~Boid() {}
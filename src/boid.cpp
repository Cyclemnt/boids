#include "../include/boid.hpp"
#include "boid.hpp"
#include <cmath>
#include <iostream>

Boid::Boid(vPose pose_, int fov_, double speed_, double angVelocity_)
    : timeStep(64), pose(pose_), fov(fov_), speed(speed_), angVelocity(angVelocity_), currentInteraction(Interaction::COHESION) {}

// Setters
void Boid::setTimeStep(int timeStep_) {
    timeStep = timeStep_;
}

void Boid::move(int envWidth, int envHeight) {
    double timeStepInSeconds = static_cast<double>(timeStep) / 1000.0;
    pose.x += (speed * timeStepInSeconds * cos(pose.theta));
    pose.y += (speed * timeStepInSeconds * sin(pose.theta));

    if (pose.x < 0) {
        pose.x += envWidth;
    } else if (pose.x >= envWidth) {
        pose.x -= envWidth;
    }

    if (pose.y < 0) {
        pose.y += envHeight;
    } else if (pose.y >= envHeight) {
        pose.y -= envHeight;
    }
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
    double timeStepInSeconds = static_cast<double>(timeStep) / 1000.0;
    pose.theta += dir * (angVelocity * timeStepInSeconds);
    pose.theta = fmod(pose.theta, 2*M_PI);
}

vPose Boid::getPose() const {
    return pose;
}
Interaction Boid::getCurrentInteraction() const {
    return currentInteraction;
}

Boid::~Boid() {}
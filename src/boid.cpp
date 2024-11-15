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

    // Assurer le comportement torique de l'environnement
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

void Boid::applyRules(Interaction interaction, std::vector<Boid*> neighborboids) {
        
        if (!neighborboids.empty()) {
            currentInteraction = interaction;


        vPose avgPose = {0, 0, 0};

        for (int i = 0; i < neighborboids.size(); i++) {
            avgPose = avgPose + neighborboids[i]->getPose();
            
        }
        avgPose = avgPose / neighborboids.size();
        vPose relPose = avgPose-pose;
        int dir=0;
        std::cout << relPose.x << std::endl;
        if (interaction == Interaction::DISTANCING) {
            if (fmod(atan2(relPose.y, relPose.x) + M_PI,2*M_PI) < fmod(pose.theta, 2*M_PI)){
                dir = -1;
            }
            else {
                dir = 1;
            }
        }
        else if (interaction == Interaction::ALIGNMENT) {
            if (fmod(pose.theta-avgPose.theta, M_PI) > 1e-6) {
                dir = -1;
            }
            else if (fmod(pose.theta-avgPose.theta, M_PI) < 1e-6){
                dir = 1;
            }
        }
        else if (interaction == Interaction::COHESION) {
            if (fmod(atan2(relPose.y, relPose.x),2*M_PI) < fmod(pose.theta, 2*M_PI))
            dir = 1;

            else {
                dir = -1;
            }
        }
        double timeStepInSeconds = static_cast<double>(timeStep) / 1000.0;
        pose.theta += dir * (angVelocity * timeStepInSeconds);
        pose.theta = fmod(pose.theta, 2*M_PI);
        }
}

vPose Boid::getPose() const {
    return pose;
}
Interaction Boid::getCurrentInteraction() const {
    return currentInteraction;
}

Boid::~Boid() {}
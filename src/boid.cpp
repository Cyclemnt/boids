#include "../include/boid.hpp"
#include "boid.hpp"
#include <iostream>
#include <cmath>
#include <algorithm> // std::clamp()

Boid::Boid(vPose pose_, double speed_, double angVelocity_, int lifeTime_)
    : pose(pose_), speed(speed_), angVelocity(angVelocity_), lifeTime(lifeTime_), currentInteraction(Interaction::NONE), timeStep(64) {}

// Setters
void Boid::setTimeStep(int timeStep_) {
    timeStep = timeStep_;
}

void Boid::setSpeed(double speed_) {
    speed = speed_;
}

void Boid::setAngVelocity(double angVelocity_) {
    angVelocity = angVelocity_;
}

void Boid::setLifeTime(int lifeTime_) {
    lifeTime = lifeTime_;
}

// Méthode pour faire avancer le boid
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

// Méthode pour modifier l'orientation du boid en fonction des voisins
void Boid::applyRules(Interaction interaction, std::vector<Boid*> neighbors) {
    currentInteraction = interaction; // Mettre à jour l'interaction actuelle

    if (interaction == Interaction::NONE || neighbors.empty()) {
        return; // Aucune règle à appliquer
    }

    // Calcul de la position moyenne des voisins
    vPose avgPose = {0, 0, 0};
    double avgThetaX = 0;
    double avgThetaY = 0;

    for (const Boid* neighbor : neighbors) {
        avgPose = avgPose + neighbor->getPose();
        avgThetaX += cos(neighbor->getPose().theta);
        avgThetaY += sin(neighbor->getPose().theta);
    }
    avgPose = avgPose / neighbors.size();
    avgThetaX = avgThetaX / neighbors.size();
    avgThetaY = avgThetaY / neighbors.size();

    // Calcul de la direction cible en fonction de l'interaction
    double targetTheta = 0;
    if (interaction == Interaction::DISTANCING) {
        vPose relPose = avgPose - pose;
        targetTheta = atan2(-relPose.y, -relPose.x);  // Éloignement, direction opposée au centre
    } else if (interaction == Interaction::ALIGNMENT) {
        targetTheta = atan2(avgThetaY, avgThetaX);    // Alignement avec l'orientation moyenne
    } else if (interaction == Interaction::COHESION) {
        vPose relPose = avgPose - pose;
        targetTheta = atan2(relPose.y, relPose.x); // Cohésion, direction vers le centre
    } else if (interaction == Interaction::FLED) {
        vPose relPose = avgPose - pose;
        targetTheta = atan2(-relPose.y, -relPose.x);  // Éloignement, direction opposée au predator       
    }else if (interaction == Interaction::PREDATION) {
        vPose relPose = avgPose - pose;
        targetTheta = atan2(relPose.y, relPose.x); // Cohésion raproché
    }

    // Normaliser les angles entre -π et π
    double angleDifference = Types::customMod(targetTheta - pose.theta + M_PI, 2 * M_PI) - M_PI;
    // Limiter la vitesse angulaire
    double timeStepInSeconds = static_cast<double>(timeStep) / 1000.0;
    double angularChange = std::clamp(angleDifference, -angVelocity * timeStepInSeconds, angVelocity * timeStepInSeconds);
    // Mettre à jour l'orientation
    pose.theta += angularChange;
    pose.theta = Types::customMod(pose.theta, 2 * M_PI); // S'assurer que theta est dans [0, 2π[

}

// Getters
vPose Boid::getPose() const {
    return pose;
}
Interaction Boid::getCurrentInteraction() const {
    return currentInteraction;
}
double Boid::getSpeed() const {
    return speed;
}

double Boid::getAngVelocity() const {
    return angVelocity;
}

int Boid::getLifeTime() const {
    return lifeTime;
}

Boid::~Boid() {}
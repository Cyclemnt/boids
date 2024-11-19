#include "../include/boid.hpp"
#include "boid.hpp"
#include <cmath>
#include <algorithm>

Boid::Boid(vPose pose_, int fov_, double speed_, double angVelocity_)
    : pose(pose_), fov(fov_), speed(speed_), angVelocity(angVelocity_), currentInteraction(Interaction::NONE), timeStep(64) {}

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

void Boid::applyRules(Interaction interaction, std::vector<Boid*> neighbors) {
    if (neighbors.empty()) return; // Pas de voisins, aucune règle à appliquer
        
    currentInteraction = interaction;

    // Calcul de la position moyenne des voisins
    vPose avgPose = {0, 0, 0};
    for (const Boid* neighbor : neighbors) {
        avgPose = avgPose + neighbor->getPose();
    }
    avgPose = avgPose / neighbors.size();
    vPose relPose = avgPose - pose;

    // Calcul de la direction cible en fonction de l'interaction
    double targetTheta = 0;
    if (interaction == Interaction::DISTANCING) {
        targetTheta = atan2(-relPose.y, -relPose.x); // Éloignement, direction opposée
    } else if (interaction == Interaction::ALIGNMENT) {
        targetTheta = avgPose.theta; // Alignement avec l'orientation moyenne
    } else if (interaction == Interaction::COHESION) {
        targetTheta = atan2(relPose.y, relPose.x); // Cohésion, direction vers le centre
    }

    // Normaliser les angles entre -π et π
    double angleDifference = fmod(targetTheta - pose.theta + 3 * M_PI, 2 * M_PI) - M_PI;

    // Limiter la vitesse angulaire
    double angularChange = std::clamp(angleDifference, -angVelocity * timeStep, angVelocity * timeStep);

    // Mettre à jour l'orientation
    pose.theta += angularChange;
    pose.theta = fmod(pose.theta + 2 * M_PI, 2 * M_PI); // S'assurer que theta est dans [0, 2π)
}

vPose Boid::getPose() const {
    return pose;
}
Interaction Boid::getCurrentInteraction() const {
    return currentInteraction;
}
double Boid::getFOV() const {
    return fov;
}

Boid::~Boid() {}
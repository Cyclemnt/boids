#include "../include/boid.hpp"
#include "boid.hpp"
#include <cmath>
#include <algorithm> // std::clamp()

Boid::Boid(vPose pose_, double speed_, double angVelocity_)
    : pose(pose_), speed(speed_), angVelocity(angVelocity_), currentInteraction(Interaction::NONE), timeStep(64 / 1000.0) {}

// Setters
void Boid::setTimeStep(double timeStep_) {
    timeStep = timeStep_ / 1000.0; // En secondes
}

// Méthode pour faire avancer le boid
void Boid::move(int envWidth, int envHeight) {
    pose.x += (speed * timeStep * cos(pose.theta));
    pose.y += (speed * timeStep * sin(pose.theta));

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
        targetTheta = atan2(relPose.y, relPose.x);    // Cohésion, direction vers le centre
    }

    // Normaliser les angles entre -π et π
    double angleDifference = Types::customMod(targetTheta - pose.theta + M_PI, 2 * M_PI) - M_PI;
    // Limiter la vitesse angulaire
    double angularChange = std::clamp(angleDifference, -angVelocity * timeStep, angVelocity * timeStep);
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

Boid::~Boid() {}
#include "../include/boid.hpp"
#include "boid.hpp"
#include <cmath>
#include <algorithm> // std::clamp()

Boid::Boid(vPose pose_, double speed_, double angVelocity_)
    : pose(pose_), speed(speed_), angVelocity(angVelocity_), currentInteraction(Interaction::NONE), timeStep(64) {}

// Setters
void Boid::setTimeStep(int timeStep_) {
    timeStep = timeStep_;
}

// Méthode pour faire avancer le boid
void Boid::move(int envWidth, int envHeight) {
    double timeStepInSeconds = static_cast<double>(timeStep) / 1000.0;
    pose.x += (speed * timeStepInSeconds * cos(pose.theta));
    pose.y += (speed * timeStepInSeconds * sin(pose.theta));

    // Assurer le comportement torique de l'environnement
    if (pose.x < 0) pose.x += envWidth;
    else if (pose.x >= envWidth) pose.x -= envWidth;

    if (pose.y < 0) pose.y += envHeight;
    else if (pose.y >= envHeight) pose.y -= envHeight;
}

// Méthode pour modifier l'orientation du boid en fonction des voisins
void Boid::applyRules(std::vector<std::vector<Boid*>> neighbors, double weightDistancing, double weightAlignment, double weightCohesion) {

    // NONE par défaut
    currentInteraction = Interaction::NONE;
    // COHESION
    double cohesionX = 0, cohesionY = 0;
    if (!neighbors[2].empty()) {
        for (const Boid* neighbor : neighbors[2]) {
            cohesionX += neighbor->getPose().x - pose.x;
            cohesionY += neighbor->getPose().y - pose.y;
        }
        cohesionX = cohesionX / neighbors[2].size();
        cohesionY = cohesionY / neighbors[2].size();
        currentInteraction = Interaction::COHESION;
    }
    // ALIGNMENT
    double alignX = 0, alignY = 0;
    if (!neighbors[1].empty()) {
        for (const Boid* neighbor : neighbors[1]) {
            alignX += cos(neighbor->getPose().theta);
            alignY += sin(neighbor->getPose().theta);
        }
        alignX = alignX / neighbors[1].size();
        alignY = alignY / neighbors[1].size();
        currentInteraction = Interaction::ALIGNMENT;
    }
    // DISTANCING
    double distX = 0, distY = 0;
    if (!neighbors[0].empty()) {
        for (const Boid* neighbor : neighbors[0]) {
            distX -= neighbor->getPose().x - pose.x;
            distY -= neighbor->getPose().y - pose.y;
        }
        distX = distX / neighbors[0].size();
        distY = distY / neighbors[0].size();
        currentInteraction = Interaction::DISTANCING;
    }
    

    // S'il y a des voisins, changer de direction
    if (currentInteraction != Interaction::NONE) {
        // Combiner les vecteurs
        double newDirX = weightDistancing * distX + weightAlignment * alignX + weightCohesion * cohesionX;
        double newDirY = weightDistancing * distY + weightAlignment * alignY + weightCohesion * cohesionY;

        // Calculer la nouvelle orientation
        double newOrientation = atan2(newDirY, newDirX);
        // Normaliser les angles entre -π et π
        double angleDifference = Types::customMod(newOrientation - pose.theta + M_PI, 2 * M_PI) - M_PI;
        // Limiter la vitesse angulaire
        double timeStepInSeconds = static_cast<double>(timeStep) / 1000.0;
        double angularChange = std::clamp(angleDifference, -angVelocity * timeStepInSeconds, angVelocity * timeStepInSeconds);
        // Mettre à jour l'orientation
        pose.theta += angularChange;
        pose.theta = Types::customMod(pose.theta, 2 * M_PI); // S'assurer que theta est dans [0, 2π[*
    }
}

// Getters
vPose Boid::getPose() const {
    return pose;
}
Interaction Boid::getCurrentInteraction() const {
    return currentInteraction;
}

Boid::~Boid() {}
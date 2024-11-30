#include "../include/boid.hpp"
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
    } else if (pose.x > envWidth) {
        pose.x -= envWidth;
    }

    if (pose.y < 0) {
        pose.y += envHeight;
    } else if (pose.y > envHeight) {
        pose.y -= envHeight;
    }
}

// Méthode pour modifier l'orientation du boid en fonction des voisins
void Boid::applyRules(std::vector<std::vector<Boid*>> neighbors, double weightDistancing, double weightAlignment, double weightCohesion, double weightFeld, double weightPredation, double weightCatch, double weightFeed, double weightBreed, int envWidth, int envHeight) {

    // NONE par défaut
    currentInteraction = Interaction::NONE;
    // COHESION
    double cohesionX = 0, cohesionY = 0;
    if (!neighbors[2].empty()) {
        for (const Boid* neighbor : neighbors[2]) {
            double dx = neighbor->getPose().x - pose.x;
            double dy = neighbor->getPose().y - pose.y;
            if (fabs(dx) > (0.5 * envWidth)) dx -= copysign(envWidth, dx);
            if (fabs(dy) > (0.5 * envHeight)) dy -= copysign(envHeight, dy);
            cohesionX += dx;
            cohesionY += dy;
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
    // PREDATION
    double predationX = 0, predationY = 0;
    if (!neighbors[4].empty()) {
        for (const Boid* neighbor : neighbors[4]) {
            double dx = neighbor->getPose().x - pose.x;
            double dy = neighbor->getPose().y - pose.y;
            if (fabs(dx) > (0.5 * envWidth)) dx -= copysign(envWidth, dx);
            if (fabs(dy) > (0.5 * envHeight)) dy -= copysign(envHeight, dy);
            predationX += dx;
            predationY += dy;
        }
        predationX = predationX / neighbors[4].size();
        predationY = predationY / neighbors[4].size();
        currentInteraction = Interaction::PREDATION;
    }
    // FEED
    double feedX = 0, feedY = 0;
    if (!neighbors[6].empty()) {
        double minDistance = std::numeric_limits<double>::infinity();
        const Boid* closestFood = nullptr;

        for (const Boid* neighbor : neighbors[6]) {
            double dx = neighbor->getPose().x - pose.x;
            double dy = neighbor->getPose().y - pose.y;
            if (fabs(dx) > (0.5 * envWidth)) dx -= copysign(envWidth, dx);
            if (fabs(dy) > (0.5 * envHeight)) dy -= copysign(envHeight, dy);
            double distance = sqrt(dx * dx + dy * dy);

            // Mettre à jour si on trouve un food plus proche
            if (distance < minDistance) {
                minDistance = distance;
                feedX = dx;
                feedY = dy;
            }
        }
        currentInteraction = Interaction::FEED;
    }
    // DISTANCING
    double distX = 0, distY = 0;
    if (!neighbors[0].empty()) {
        for (const Boid* neighbor : neighbors[0]) {
            double dx = neighbor->getPose().x - pose.x;
            double dy = neighbor->getPose().y - pose.y;
            if (fabs(dx) > (0.5 * envWidth)) dx -= copysign(envWidth, dx);
            if (fabs(dy) > (0.5 * envHeight)) dy -= copysign(envHeight, dy);
            distX -= dx;
            distY -= dy;
        }
        distX = distX / neighbors[0].size();
        distY = distY / neighbors[0].size();
        currentInteraction = Interaction::DISTANCING;
    }
    // FLED
    double fledX = 0, fledY = 0;
    if (!neighbors[3].empty()) {
        for (const Boid* neighbor : neighbors[3]) {
            double dx = neighbor->getPose().x - pose.x;
            double dy = neighbor->getPose().y - pose.y;
            if (fabs(dx) > (0.5 * envWidth)) dx -= copysign(envWidth, dx);
            if (fabs(dy) > (0.5 * envHeight)) dy -= copysign(envHeight, dy);
            fledX -= dx;
            fledY -= dy;
        }
        fledX = fledX / neighbors[3].size();
        fledY = fledY / neighbors[3].size();
        currentInteraction = Interaction::FLED;
    }
    // CATCH
    if (!neighbors[5].empty()) {
        currentInteraction = Interaction::CATCH;
    }
    // BREED
    if (!neighbors[7].empty()) {
        currentInteraction = Interaction::BREED;
    }

    // S'il y a des voisins, changer de direction
    if (currentInteraction != Interaction::NONE) {
        // Combiner les vecteurs
        double newDirX = weightDistancing * distX + weightAlignment * alignX + weightCohesion * cohesionX + weightFeld * fledX + weightPredation * predationX + feedX * weightFeed;
        double newDirY = weightDistancing * distY + weightAlignment * alignY + weightCohesion * cohesionY + weightFeld * fledY + weightPredation * predationY + feedY * weightFeed;

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
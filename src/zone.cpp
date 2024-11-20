#include "../include/zone.hpp"
#include <cmath>

Zone::Zone(double rDistancing_, double rAlignment_, double rCohesion_, double fov_)
    : rDistancingSquared(rDistancing_ * rDistancing_), rAlignmentSquared(rAlignment_ * rAlignment_), rCohesionSquared(rCohesion_ * rCohesion_), halvedFov(fov_ / 2.0) {}

// Méthode pour obtenir tous les boids dans un certain rayon autour du boid
std::vector<Boid*> Zone::getNearBoids(Interaction interaction, Boid* boid, std::vector<Boid*> boids, int envWidth, int envHeight) {
    std::vector<Boid*> neighbors;
    double radiusSquared = 0;
    
    // Définir le rayon en fonction de l'interaction
    switch (interaction) {
        case Interaction::DISTANCING:
            radiusSquared = rDistancingSquared;
            break;
        case Interaction::ALIGNMENT:
            radiusSquared = rAlignmentSquared;
            break;
        case Interaction::COHESION:
            radiusSquared = rCohesionSquared;
            break;
        case Interaction::NONE:
            return {};
    }

    // Parcourir chaque boid pour calculer la distance torique
    for (int i = 0; i < boids.size(); i++) {
        if (boid->getPose() != boids[i]->getPose()) {
            // Calculer la distance en x en tenant compte de l'environnement torique
            double dx = std::min(std::fabs(boid->getPose().x - boids[i]->getPose().x), envWidth - std::fabs(boid->getPose().x - boids[i]->getPose().x));
            // Calculer la distance en y en tenant compte de l'environnement torique
            double dy = std::min(std::fabs(boid->getPose().y - boids[i]->getPose().y), envHeight - std::fabs(boid->getPose().y - boids[i]->getPose().y));

            // Calculer la distance euclidienne avec les distances minimales en x et y
            double distanceSquared = (dx * dx) + (dy * dy);

            // Ajouter le boid à la liste des voisins s'il est dans le rayon
            if (distanceSquared < radiusSquared && angleWithinFOV(boid->getPose(), boids[i]->getPose())) {
                neighbors.push_back(boids[i]);
            }
        }
    }
    return neighbors;
}

// Méthode pour vérifier si un boid voisin est dans le fov du boid
bool Zone::angleWithinFOV(const vPose& boidPose, const vPose& neighborPose) {
    // Calculer le vecteur directionnel du boid vers le voisin
    double dx = neighborPose.x - boidPose.x;
    double dy = neighborPose.y - boidPose.y;

    // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe X
    double angleToNeighbor = atan2(dy, dx);

    // Calculer la différence angulaire par rapport à l'orientation du boid
    double angleDifference = Types::customMod(angleToNeighbor - boidPose.theta + M_PI, 2 * M_PI) - M_PI;

    // Vérifier si la différence angulaire est dans les limites du FOV
    return std::fabs(angleDifference) <= (halvedFov);
}

Zone::~Zone() {
}

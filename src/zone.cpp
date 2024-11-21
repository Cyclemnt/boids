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

    
    vPose boidPose = boid->getPose();
    // Parcourir chaque boid pour calculer la distance torique
    for (int i = 0; i < boids.size(); i++) {
        vPose neighborPose = boids[i]->getPose();
        if (boidPose != neighborPose) {
            // Calculer la distance
            double dx = neighborPose.x - boidPose.x;
            double dy = neighborPose.y - boidPose.y;
            // Calculer la distance en tenant compte de l'environnement torique
            double dxTorus = std::min(std::fabs(dx), envWidth - std::fabs(dx));
            double dyTorus = std::min(std::fabs(dy), envHeight - std::fabs(dy));

            // Calculer la distance euclidienne avec les distances minimales en x et y
            double distanceSquared = (dxTorus * dxTorus) + (dyTorus * dyTorus);
            
            // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe x
            double angleToNeighbor = atan2(dy, dx);
            // Calculer la différence angulaire par rapport à l'orientation du boid
            double angleDifference = Types::customMod(angleToNeighbor - boidPose.theta + M_PI, 2 * M_PI) - M_PI;
            bool isWithinFOV = std::fabs(angleDifference) <= (halvedFov);

            // Ajouter le boid à la liste des voisins s'il est dans le rayon
            if (distanceSquared < radiusSquared && isWithinFOV) {
                neighbors.push_back(boids[i]);
            }
        }
    }
    return neighbors;
}

Zone::~Zone() {
}

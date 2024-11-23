#include "../include/zone.hpp"
#include "../include/types.hpp"
#include <cmath>

Zone::Zone(double rDistancing_, double rAlignment_, double rCohesion_, double fov_)
    : rDistancing(rDistancing_), rAlignment(rAlignment_), rCohesion(rCohesion_), fov(fov_) {}

// Méthode pour obtenir tous les boids dans un certain rayon autour du boid
std::vector<std::vector<Boid*>> Zone::getNearBoids(Boid* boid, std::vector<Boid*> boids, int envWidth, int envHeight) {
    // Initialiser les tableaux de sortie
    std::vector<Boid*> distancingNeighbors = {};
    std::vector<Boid*> alignmentNeighbors = {};
    std::vector<Boid*> cohesionNeighbors = {};

    vPose boidPose = boid->getPose();

    // Parcourir chaque boid pour calculer la distance torique
    for (int i = 0; i < boids.size(); i++) {
        if (boid == boids[i]) continue; // Ne pas prendre en compte le boid même

        vPose neighborPose = boids[i]->getPose();

        // Calculer la distance
        double dx = neighborPose.x - boidPose.x;
        double dy = neighborPose.y - boidPose.y;

        double absDx = fabs(dx);
        double absDy = fabs(dy);

        // Calculer la distance torique
        if (absDx > 0.5 * envWidth) absDx = envWidth - absDx;
        if (absDy > 0.5 * envHeight) absDy = envHeight - absDy;
        double distance = sqrt((absDx * absDx) + (absDy * absDy));

        // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe x
        double angleToNeighbor = atan2(dy, dx);
        // Calculer la différence angulaire par rapport à l'orientation du boid
        double angleDifference = Types::customMod(angleToNeighbor - boidPose.theta + M_PI, 2 * M_PI) - M_PI;
        bool isWithinFOV = (fabs(angleDifference) <= (fov / 2.0));

        if (!isWithinFOV) continue; // Ne pas prendre en compte les boids hors du fov

        // Ajouter le boid à la liste des voisins s'il est dans le rayon
        if (distance < rDistancing) {
            distancingNeighbors.push_back(boids[i]);
        }
        if (distance < rAlignment) {
            alignmentNeighbors.push_back(boids[i]);
        }
        if (distance < rCohesion) {
            cohesionNeighbors.push_back(boids[i]);
        }
    }
    return {distancingNeighbors, alignmentNeighbors, cohesionNeighbors};
}

Zone::~Zone() {
}

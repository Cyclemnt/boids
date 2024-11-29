#include "../include/zone.hpp"
#include "../include/types.hpp"
#include <cmath>

Zone::Zone(double rDistancing_, double rAlignment_, double rCohesion_, double rFollow_, double fov_)
    : rDistancing(rDistancing_), rAlignment(rAlignment_), rCohesion(rCohesion_),rFollow(rFollow_), fov(fov_) {}

// Méthode pour obtenir tous les boids dans un certain rayon autour du boid
std::vector<std::vector<Boid*>> Zone::getNearBoids(Boid* boid,Boid* mouse, std::vector<Boid*> boids, int envWidth, int envHeight,bool mouseON) {
    // Initialiser les tableaux de sortie
    std::vector<Boid*> distancingNeighbors = {};
    std::vector<Boid*> alignmentNeighbors = {};
    std::vector<Boid*> cohesionNeighbors = {};
    std::vector<Boid*> followNeighbors = {};
    vPose boidPose = boid->getPose();

    // Parcourir chaque boid pour calculer la distance torique
    for (int i = 0; i < boids.size(); i++) {
        if (boid == boids[i]) continue; // Ne pas prendre en compte le boid même

        vPose neighborPose = boids[i]->getPose();

        // Calculer la distance
        double dx = neighborPose.x - boidPose.x;
        double dy = neighborPose.y - boidPose.y;

        // Calculer la distance torique
        if (fabs(dx) > (0.5 * envWidth)) dx -= copysign(envWidth, dx);
        if (fabs(dy) > (0.5 * envHeight)) dy -= copysign(envHeight, dy);

        double distance = sqrt((dx * dx) + (dy * dy));


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
        if (!mouseON){
            dx= std::min(std::fabs(boid->getPose().x -mouse->getPose().x), envWidth - std::fabs(boid->getPose().x - mouse->getPose().x));
            // Calculer la distance en y en tenant compte de l'environnement torique
            dy = std::min(std::fabs(boid->getPose().y - mouse->getPose().y), envHeight - std::fabs(boid->getPose().y - mouse->getPose().y));   
            distance = sqrt((dx * dx) + (dy * dy));
            if (distance < rFollow) {   
                followNeighbors.push_back(mouse);
            }
        }
    }
    return {distancingNeighbors, alignmentNeighbors, cohesionNeighbors,followNeighbors};
}

Zone::~Zone() {
}

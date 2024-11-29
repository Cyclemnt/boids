#include "../include/zone.hpp"
#include <cmath>

Zone::Zone(double rDistancing_, double rAlignment_, double rCohesion_, double rPredation_, double rFled_, double rCatch_, double fov_, double instinct_)
    : rDistancing(rDistancing_), rAlignment(rAlignment_), rCohesion(rCohesion_), rPredation(rPredation_), rFled(rFled_), rCatch(rCatch_), fov(fov_), instinct(instinct_) {}

// Méthode pour obtenir tous les boids dans un certain rayon autour du boid
std::vector<std::vector<Boid*>> Zone::getNearBoids(Boid* boid, std::vector<Boid*> boids, std::vector<Boid*> predators, int envWidth, int envHeight) {
    // Initialiser les tableaux de sortie
    std::vector<Boid*> distancingNeighbors = {};
    std::vector<Boid*> alignmentNeighbors = {};
    std::vector<Boid*> cohesionNeighbors = {};
    std::vector<Boid*> fledNeighbors = {};
    std::vector<Boid*> predationNeighbors = {};
    std::vector<Boid*> catchNeighbors = {};

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
        if (distance < rPredation) {
            predationNeighbors.push_back(boids[i]);
        }
    }
    // Parcourir chaque predator pour calculer la distance torique
    for (int i = 0; i < predators.size(); i++) {
        if (boid == predators[i]) continue; // Ne pas prendre en compte le boid même

        vPose neighborPose = predators[i]->getPose();

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
        // Calculer la différence angulaire par rapport à l'orientation du predator
        double angleDifference = Types::customMod(angleToNeighbor - boidPose.theta + M_PI, 2 * M_PI) - M_PI;
        bool isWithinInstinct = (fabs(angleDifference) <= (fov / 2.0));

        if (!isWithinInstinct) continue; // Ne pas prendre en compte les predators hors de l'instinct

        // Ajouter le predator à la liste des voisins s'il est dans le rayon
        if (distance < rFled) {
            fledNeighbors.push_back(predators[i]);
        }
        if (distance < rCatch) {
            catchNeighbors.push_back(predators[i]);
        }
    }
    return {distancingNeighbors, alignmentNeighbors, cohesionNeighbors, feldNeighbors, predationNeighbors, catchNeighbors};
}


// Méthode pour vérifier si un boid voisin est dans le fov du boid
bool Zone::isWithinFOV(const vPose& boidPose, const vPose& neighborPose) {
    // Calculer le vecteur directionnel du boid vers le voisin
    double dx = neighborPose.x - boidPose.x;
    double dy = neighborPose.y - boidPose.y;

    // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe X
    double angleToNeighbor = atan2(dy, dx);

    // Calculer la différence angulaire par rapport à l'orientation du boid
    double angleDifference = Types::customMod(angleToNeighbor - boidPose.theta + M_PI, 2 * M_PI) - M_PI;

    // Vérifier si la différence angulaire est dans les limites du FOV
    return std::fabs(angleDifference) <= (fov / 2);
}

// Méthode pour vérifier si un predator voisin est dans le fov (instinct) du boid
bool Zone::isWithinInstinct(const vPose& boidPose, const vPose& neighborPose) {
    // Calculer le vecteur directionnel du boid vers le voisin
    double dx = neighborPose.x - boidPose.x;
    double dy = neighborPose.y - boidPose.y;

    // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe X
    double angleToNeighbor = atan2(dy, dx);

    // Calculer la différence angulaire par rapport à l'orientation du boid
    double angleDifference = Types::customMod(angleToNeighbor - boidPose.theta + M_PI, 2 * M_PI) - M_PI;

    // Vérifier si la différence angulaire est dans les limites du FOV (instinct)
    return std::fabs(angleDifference) <= (instinct / 2);
}

Zone::~Zone() {
}

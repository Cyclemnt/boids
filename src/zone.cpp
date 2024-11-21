#include "../include/zone.hpp"
#include <cmath>

Zone::Zone(double rDistancing_, double rAlignment_, double rCohesion_, double rPredation_, double rFled_, double fov_, double instinct_)
    : rDistancing(rDistancing_), rAlignment(rAlignment_), rCohesion(rCohesion_), rPredation(rPredation_), rFled(rFled_), fov(fov_), instinct(instinct_) {}

// Méthode pour obtenir tous les boids dans un certain rayon autour du boid
std::vector<Boid*> Zone::getNearBoids(Interaction interaction, Boid* boid, std::vector<Boid*> boids, Boid* target, std::vector<Boid*> targets, Boid* predator, std::vector<Boid*> predators, int envWidth, int envHeight) {
    std::vector<Boid*> neighbors;
    double radius = 0;
    
    // Définir le rayon en fonction de l'interaction
    switch (interaction) {
        case Interaction::DISTANCING:
            radius = rDistancing;
            break;
        case Interaction::FLED:
            radius = rFled;
            break;
        case Interaction::ALIGNMENT:
            radius = rAlignment;
            break;
        case Interaction::PREDATION:
            radius = rPredation;
            break;
        case Interaction::COHESION:
            radius = rCohesion;
            break;
        case Interaction::NONE:
            return {};
    }

    // Parcourir chaque cible pour calculer la distance torique
    for (int i = 0; i < targets.size(); i++) {
        if (boid->getPose() != targets[i]->getPose()) {
            // Calculer la distance en x en tenant compte de l'environnement torique
            double dx = std::min(std::fabs(boid->getPose().x - targets[i]->getPose().x), 
                             envWidth - std::fabs(boid->getPose().x - targets[i]->getPose().x));
            double dy = std::min(std::fabs(boid->getPose().y - targets[i]->getPose().y), 
                             envHeight - std::fabs(boid->getPose().y - targets[i]->getPose().y));

            // Calculer la distance euclidienne avec les distances minimales en x et y
            double distance = sqrt((dx * dx) + (dy * dy));

            // Vérifier si la cible est un predator
            bool isPredator = false;
        for (int j = 0; j < predators.size(); j++) {
            if (predators[j] == targets[i]) {
                isPredator = true;
                break;
            }
        }

        // Ajouter à la liste des voisins en fonction de l'interaction
        if (isPredator && distance < radius && angleWithinInstinct(boid->getPose(), targets[i]->getPose())) {
            neighbors.push_back(targets[i]);
        } else if (!isPredator && distance < radius && angleWithinFOV(boid->getPose(), targets[i]->getPose())) {
            neighbors.push_back(targets[i]);
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
    return std::fabs(angleDifference) <= (fov / 2);
}

// Méthode pour vérifier si un predator voisin est dans le fov (instinct) du boid
bool Zone::angleWithinInstinct(const vPose& boidPose, const vPose& neighborPose) {
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

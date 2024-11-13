#include "../include/zone.hpp"
#include <cmath>
#include <iostream>
#include <cmath>

Zone::Zone(double rDistancing_, double rAlignment_, double rCohesion_)
    : rDistancing(rDistancing_), rAlignment(rAlignment_), rCohesion(rCohesion_) {}

std::vector<Boid*> Zone::getNearBoids(Interaction interaction, Boid* boid, std::vector<Boid*> boids, int envWidth, int envHeight) {
    std::vector<Boid*> neighbors;
    double radius = 0;

    // Définir le rayon en fonction de l'interaction
    switch (interaction) {
        case Interaction::DISTANCING:
            radius = rDistancing;
            break;
        case Interaction::ALIGNMENT:
            radius = rAlignment;
            break;
        case Interaction::COHESION:
            radius = rCohesion;
            break;
    }

    // Parcourir chaque boid pour calculer la distance torique
    for (int i = boids.size() - 1; i >= 0; i--) {
        // Calculer la distance en x en tenant compte de l'environnement torique
        double dx = std::fabs(boid->getPose().x - std::min(boids[i]->getPose().x, envWidth - boids[i]->getPose().x));
        // Calculer la distance en y en tenant compte de l'environnement torique
        double dy = std::fabs(boid->getPose().y - std::min(boids[i]->getPose().y, envHeight - boids[i]->getPose().y));

        // Calculer la distance euclidienne avec les distances minimales en x et y
        double distance = sqrt((dx * dx) + (dy * dy));

        // Ajouter le boid à la liste des voisins s'il est dans le rayon
        if (distance < radius) {
            neighbors.push_back(boids[i]);
        }
    }

    return neighbors;
}

Zone::~Zone() {
}

#include "../include/zone.hpp"
#include <cmath>
#include <iostream>

Zone::Zone(double rDistancing_, double rAlignment_, double rCohesion_)
    : rDistancing(rDistancing_), rAlignment(rAlignment_), rCohesion(rCohesion_) {}

std::vector<Boid*> Zone::getNearBoids(Interaction interaction, Boid* boid, std::vector<Boid*> boids) {
    std::vector<Boid*> neighbors;
    double radius = 0;
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

    for (int i = boids.size() - 1; i > 0; i--) {
        double dx = std::fabs(boid->getPose().x - boids[i]->getPose().x);
        double dy = std::fabs(boid->getPose().y - boids[i]->getPose().y);
        double distance = sqrt((dx * dx) + (dy * dy));
        if (distance < radius) {
            neighbors.push_back(boids[i]);
        }
    }
    return neighbors;
}

Zone::~Zone() {
}

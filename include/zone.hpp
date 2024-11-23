#ifndef ZONE_HPP
#define ZONE_HPP

#include "boid.hpp"

class Zone
{
private : 
    double rDistancing, rAlignment, rCohesion, fov;

public : 
    Zone(double rDistancing_, double rAlignment_, double rCohesion_, double fov_);

    // Méthode pour obtenir tous les boids dans un certain rayon autour du boid
    std::vector<std::vector<Boid*>> getNearBoids(Boid* boid, std::vector<Boid*> boids, int envWidth, int envHeight);

    ~Zone();
};

#endif // ZONE_HPP
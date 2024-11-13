#ifndef ZONE_HPP
#define ZONE_HPP

#include "boid.hpp"
#include "types.hpp"
#include <utility>
#include <vector>

using Types::Interaction;

class Zone
{
private : 
    double rDistancing, rAlignment, rCohesion;

public : 
    Zone(double rDistancing_, double rAlignment_, double rCohesion_);

    std::vector<Boid*> getNearBoids(Interaction interaction, Boid* boid, std::vector<Boid*> boids, int envWidth, int envHeight);

    ~Zone();
};

#endif // ZONE_HPP
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
    Zone(double rDistancing, double rAlignment, double rCohesion);

    std::vector<Boid*> getNearBoids(Interaction interaction, Boid* boid, std::vector<Boid*> boids);

    ~Zone();

};

#endif // ZONE_HPP
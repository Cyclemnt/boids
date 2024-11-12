#include "zone.hpp"


Zone::Zone(double rDistancing, double rAlignment, double rCohesion)
{
}

std::vector<Boid *> Zone::getSeparationNeighbors(Boid* boid, std::vector<Boid*> boids)
{
    return std::vector<Boid *>();
}

std::vector<Boid *> Zone::getAlignmentNeighbors(Boid* boid, std::vector<Boid*> boids)
{
    return std::vector<Boid *>();
}

std::vector<Boid *> Zone::getCohesionNeighbors(Boid* boid, std::vector<Boid*> boids)
{
    return std::vector<Boid *>();
}

Zone::~Zone()
{
}

#include "../include/simulation.hpp"

Simulation::Simulation(double width_,double height_)
{
}

void Simulation::addBoid()
{
}

void Simulation::removeBoid()
{
}

void Simulation::reset()
{
}

void Simulation::update()
{
}

void Simulation::togglePause()
{
}

bool Simulation::isPaused() const
{
    return false;
}

void Simulation::sendInfoToBoids()
{
}

Simulation::~Simulation() {
    for (int i = boids.size(); i > 0 ; i--) {
        delete boids[i];
        boids.pop_back();
    }
}

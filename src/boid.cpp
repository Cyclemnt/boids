#include "../include/boid.hpp"
#include "boid.hpp"

Boid::Boid(float x, float y, float angle)
{
}

std::pair<float, float> Boid::getPosition() const
{
    return std::pair<float, float>();
}

float Boid::getDirection() const
{
    return 0.0f;
}

void Boid::move()
{
}

void Boid::applyRules(const std::vector<Boid> &neighbors)
{
}

void Boid::updateColor()
{
}

void Boid::wrapPosition(float width, float height)
{
}

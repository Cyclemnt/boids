#include "../include/zone.hpp"

Zone::Zone(float distancingRadius, float alignmentRadius, float cohesionRadius)
{
}

float Zone::getDistancingRadius() const
{
    return 0.0f;
}

float Zone::getAlignmentRadius() const
{
    return 0.0f;
}

float Zone::getCohesionRadius() const
{
    return 0.0f;
}

void Zone::setDistancingRadius(float radius)
{
}

void Zone::setAlignmentRadius(float radius)
{
}

void Zone::setCohesionRadius(float radius)
{
}

bool Zone::isInDistancingZone(float distance) const
{
    return false;
}

bool Zone::isInAlignmentZone(float distance) const
{
    return false;
}

bool Zone::isInCohesionZone(float distance) const
{
    return false;
}

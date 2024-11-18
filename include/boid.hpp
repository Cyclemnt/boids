#ifndef BOID_HPP
#define BOID_HPP

#include "types.hpp"
#include <vector>

using Types::vPose;
using Types::Interaction;

class Boid
{
private:
    int timeStep; 
    vPose pose;
    int fov;
    double speed, angVelocity;
    Interaction currentInteraction;

public:
    Boid(vPose pose_, int fov_, double speed_, double angVelocity_);

    void setTimeStep(int timeStep_);

    void move(int envWidth, int envHeight);
    void applyRules(Interaction interaction, std::vector<Boid*> neighbors);

    vPose getPose() const;
    Interaction getCurrentInteraction() const;
    
    ~Boid();
};

#endif // BOID_HPP
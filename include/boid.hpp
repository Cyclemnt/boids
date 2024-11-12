#ifndef BOID_HPP
#define BOID_HPP

#include "types.hpp"
#include <utility>
#include <vector>

using Types::vPose;
using Types::Interaction;

class Boid
{
private:
    int timeStep; 
    vPose pose;
    int fov;
    double maxSpeed, maxAngVelocity;
    Interaction currentInteraction;

public:
    Boid(vPose pose_, int fov_, double maxSpeed_, double maxAngVelocity_);

    void setTimeStep(int timeStep_);

    void SeparationCorrection(std::vector<Boid*> neighborsSeparation);
    void AlignmentCorrection(std::vector<Boid*> neighborsAlignment);
    void CohesionCorrection(std::vector<Boid*> neighborsCohesion);

    vPose getPose() const;
    Interaction getCurrentInteraction() const;
    
    ~Boid();
};

#endif // BOID_HPP
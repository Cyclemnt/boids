#ifndef BOID_HPP
#define BOID_HPP

#include "types.hpp"
#include <vector>

using Types::vPose;
using Types::Interaction;

class Boid
{
private:
    double timeStep; // timeStep, précalculé en secondes
    vPose pose;
    double speed, angVelocity;
    Interaction currentInteraction;

public:
    Boid(vPose pose_, double speed_, double angVelocity_);
    
    // Setters
    void setTimeStep(double timeStep_);

    // Méthode pour faire avancer le boid
    void move(int envWidth, int envHeight);
    // Méthode pour modifier l'orientation du boid en fonction des voisins
    void applyRules(Interaction interaction, std::vector<Boid*> neighbors);

    // Getters
    vPose getPose() const;
    Interaction getCurrentInteraction() const;
    
    ~Boid();
};

#endif // BOID_HPP
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
    double speed, angVelocity;
    Interaction currentInteraction;

public:
    Boid(vPose pose_, double speed_, double angVelocity_);
    // Setter
    void setTimeStep(int timeStep_);

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
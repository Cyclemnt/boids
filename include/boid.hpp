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
    
    // Setters
    void setTimeStep(int timeStep_);

    // Méthode pour faire avancer le boid
    void move(int envWidth, int envHeight);
    // Méthode pour modifier l'orientation du boid en fonction des voisins
    void applyRules(std::vector<std::vector<Boid*>> neighbors, double weightDistancing, double weightAlignment, double weightCohesion, int envWidth, int envHeight);

    // Getters
    vPose getPose() const;
    Interaction getCurrentInteraction() const;
    
    ~Boid();
};

#endif // BOID_HPP
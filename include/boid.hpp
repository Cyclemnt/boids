#ifndef BOID_HPP
#define BOID_HPP

#include "types.hpp"
#include <vector>

using Types::vPose;
using Types::Interaction;

class Boid
{
private:
    int timeStep, lifeTime; 
    vPose pose;
    double speed, angVelocity;
    Interaction currentInteraction;

public:
    Boid(vPose pose_, double speed_, double angVelocity_, int lifeTime_);
    
    // Setters
    void setTimeStep(int timeStep_);
    void setSpeed(double speed_);
    void setAngVelocity(double angVelocity_);
    void setLifeTime(int lifeTime_);
    // Méthode pour faire avancer le boid
    void move(int envWidth, int envHeight);
    // Méthode pour modifier l'orientation du boid en fonction des voisins
    void applyRules(std::vector<Boid*> neighbors, double weightDistancing, double weightAlignment, double weightCohesion, double weightFeld, double weightPredation, double weightCatch, int envWidth, int envHeight);

    // Getters
    vPose getPose() const;
    Interaction getCurrentInteraction() const;
    double getSpeed() const;
    double getAngVelocity() const;
    int getLifeTime() const;

    
    ~Boid();
};

#endif // BOID_HPP
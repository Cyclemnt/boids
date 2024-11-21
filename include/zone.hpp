#ifndef ZONE_HPP
#define ZONE_HPP

#include "boid.hpp"
#include "types.hpp"

using Types::Interaction;

class Zone
{
private : 
    double rDistancing, rAlignment, rCohesion,rFollow, instinct, fov;

public : 
    //Constructeur
    Zone(double rDistancing_, double rAlignment_, double rCohesion_,double rFollow_, double fov_, double instinct);
    // Méthode pour obtenir tous les boids dans un certain rayon autour du boid
    std::vector<Boid*> getNearBoids(Interaction interaction, Boid* boid, std::vector<Boid*> boids, Boid* mouse, int envWidth, int envHeight);
    // Méthode pour vérifier si un boid voisin est dans le fov du boid
    bool angleWithinFOV(const vPose& boidPose, const vPose& neighborPose);

    ~Zone();
};

#endif // ZONE_HPP
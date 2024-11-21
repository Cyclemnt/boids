#ifndef ZONE_HPP
#define ZONE_HPP

#include "boid.hpp"
#include "types.hpp"

using Types::Interaction;

class Zone
{
private : 
    double rDistancing, rAlignment, rCohesion, rPredation, rFled, fov, instinct;

public : 
    Zone(double rDistancing_, double rAlignment_, double rCohesion_, double rPredation_, double rFled_, double fov_, double instinct_);

    // Méthode pour obtenir tous les boids dans un certain rayon autour du boid
    std::vector<Boid*> getNearBoids(Interaction interaction, Boid* boid, std::vector<Boid*> boids, Boid* target, std::vector<Boid*> targets, Boid* predator, std::vector<Boid*> predators, int envWidth, int envHeight);
    // Méthode pour vérifier si un boid voisin est dans le fov du boid
    bool angleWithinFOV(const vPose& boidPose, const vPose& neighborPose);
    // Méthode pour vérifier si un predator voisin est dans le fov(instinct) du boid
    bool angleWithinInstinct(const vPose& boidPose, const vPose& neighborPose);

    ~Zone();
};

#endif // ZONE_HPP
#ifndef ZONE_HPP
#define ZONE_HPP

#include "boid.hpp"
#include "types.hpp"

using Types::Interaction;

class Zone
{
private : 
    double rDistancing, rAlignment, rCohesion, rPredation, rFled, rCatch, rFeed, rBreed, fov, instinct;

public : 
    Zone(double rDistancing_, double rAlignment_, double rCohesion_, double rPredation_, double rFled_, double rCatch_, double rFeed_, double rBreed_, double fov_, double instinct_);

    // Méthode pour obtenir tous les boids dans un certain rayon autour du boid
    std::vector<std::vector<Boid*>> getNearBoids(Boid* boid, std::vector<Boid*> boids, std::vector<Boid*> predators, std::vector<Boid*> foods, int envWidth, int envHeight);
    // Méthode pour vérifier si un boid voisin est dans le fov du boid
    bool isWithinFOV(const vPose& boidPose, const vPose& neighborPose);
    // Méthode pour vérifier si un predator voisin est dans le fov(instinct) du boid
    bool isWithinInstinct(const vPose& boidPose, const vPose& neighborPose);

    ~Zone();
};

#endif // ZONE_HPP
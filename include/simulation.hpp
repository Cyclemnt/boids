#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>
#include "../include/boid.hpp"

class Simulation {
public:
    // Constructeur et destructeur
    Simulation(float width, float height);
    ~Simulation();

    // Méthodes de gestion de la simulation
    void addBoid();
    void removeBoid();
    void reset();
    void update();
    void togglePause();

    // Accesseurs
    bool isPaused() const;

private:
    // Attributs principaux
    std::vector<Boid> boids;
    float width, height;  // Dimensions de la zone de simulation
    bool paused;          // État de la simulation

    // Paramètres ajustables
    float viewAngle;
    float distancingRadius;
    float alignmentRadius;
    float cohesionRadius;
    float maxSpeed;
    float maxTurnRate;

    // Méthodes privées
    void applyRulesToBoid(Boid& boid);
    std::vector<Boid> getNeighbors(const Boid& boid) const;
};

#endif // SIMULATION_HPP

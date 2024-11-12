#ifndef BOID_HPP
#define BOID_HPP

#include <vector>
#include <utility> // for std::pair

class Boid {
private:
    // Attributs principaux
    float x, y;          // Position
    float vx, vy;        // Vitesse
    float direction;     // Angle de déplacement
    float viewAngle;     // Angle de vue

    // État d'interaction pour couleur
    enum Interaction { DISTANCING, ALIGNMENT, COHESION };
    Interaction currentInteraction;
    
    // Paramètres de mouvement
    float maxSpeed;
    float maxTurnRate;
    
public:
    // Constructeur
    Boid(float x, float y, float angle);

    // Accesseurs pour position et direction
    std::pair<float, float> getPosition() const;
    float getDirection() const;

    // Méthodes de mise à jour
    void move();
    void applyRules(const std::vector<Boid>& neighbors);
    void updateColor();

    // Gestion des bords
    void wrapPosition(float width, float height);
};

#endif // BOID_HPP

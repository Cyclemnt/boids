#ifndef ZONE_HPP
#define ZONE_HPP

class Zone {
private:
    // Attributs de zones
    float distancingRadius;
    float alignmentRadius;
    float cohesionRadius;
    
public:
    // Constructeur
    Zone(float distancingRadius, float alignmentRadius, float cohesionRadius);

    // Accesseurs
    float getDistancingRadius() const;
    float getAlignmentRadius() const;
    float getCohesionRadius() const;

    // Mutateurs pour ajuster les zones
    void setDistancingRadius(float radius);
    void setAlignmentRadius(float radius);
    void setCohesionRadius(float radius);

    // Méthodes de vérification des zones
    bool isInDistancingZone(float distance) const;
    bool isInAlignmentZone(float distance) const;
    bool isInCohesionZone(float distance) const;
};

#endif // ZONE_HPP

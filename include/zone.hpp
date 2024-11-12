#include <utility>
#include<vector>
#include"boid.hpp"
class Zone {
private : 
    double rDistancing, rAlignment, rCohesion;
    
public : 
    Zone(double rDistancing, double rAlignment, double rCohesion);

    std::vector<Boid*> getSeparationNeighbors(Boid* boid, std::vector<Boid*> boids);
    std::vector<Boid*> getAlignmentNeighbors(Boid* boid, std::vector<Boid*> boids);
    std::vector<Boid*> getCohesionNeighbors(Boid* boid, std::vector<Boid*> boids);

    ~Zone();

};
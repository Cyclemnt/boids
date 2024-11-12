#include <utility>

class Boid
{
private:
    int timeStep; 
    std::pair<double, double> position;
    double orientation;
    int fov;
    double rSeparation, rAlignment, rCohesion;
    double maxSpeed, maxAngVelocity;
public:
    Boid(/* args */);

    void setTimeStep(int timeStep_);

    ~Boid();
};
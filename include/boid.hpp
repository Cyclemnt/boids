#include <utility>

struct vPose {
    double x, y, theta;
};

enum class Interaction { DISTANCING, ALIGNMENT, COHESION };

class Boid
{
private:
    int timeStep; 
    vPose pose;
    int fov;
    double maxSpeed, maxAngVelocity;
    Interaction currentInteraction;

public:
    Boid(vPose pose_, int fov_, double maxSpeed_, double maxAngVelocity_);

    void setTimeStep(int timeStep_);

    ~Boid();
};
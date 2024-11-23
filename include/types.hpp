#ifndef TYPES_HPP
#define TYPES_HPP

#include <cmath>
#include <vector>

namespace Types
{
    enum class Interaction { DISTANCING, ALIGNMENT, COHESION, NONE };

    struct BoidData {
        std::vector<float> positionsX;
        std::vector<float> positionsY;
        std::vector<float> orientations;
        std::vector<Interaction> interactions;

        float* d_positionsX = nullptr;
        float* d_positionsY = nullptr;
        float* d_orientations = nullptr;
        Interaction* d_interations = nullptr;

        float speed;
        float angVelocity;
        float halvedFov;
        float timeStep;
        float rDistancingSquared;
        float rAlignmentSquared;
        float rCohesionSquared;
    };

    // Modulo customisé
    inline float customMod(float a, float n) {return fmodf(fmodf(a, n) + n, n);}

} // namespace Types

#endif // TYPES_HPP
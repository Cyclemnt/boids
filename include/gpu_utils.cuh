#ifndef GPU_UTILS_CUH
#define GPU_UTILS_CUH

#include "types.hpp"
#include <cuda_runtime.h>

// Fonction pour allouer la mémoire GPU
void allocateBoidDataOnGPU(Types::BoidData& boids);

// Fonction pour libérer la mémoire GPU
void freeBoidDataOnGPU(Types::BoidData& boids);

// Fonction pour transférer les données CPU -> GPU
void copyBoidDataToGPU(const Types::BoidData& boids);

// Fonction pour transférer les données GPU -> CPU
void copyBoidDataToCPU(Types::BoidData& boids);

// Fonction pour réallouer si la taille change
void reallocateIfNecessary(Types::BoidData& boids);

// kernel
__global__ void updateBoidsKernel(
    float* positionsX, float* positionsY, float* orientations, Types::Interaction* interactions,
    const int numBoids, const float envWidth, const float envHeight, const float speed, const float angVelocity, const float timeStep,
    const float halvedFov, const float rDistancingSquared, const float rAlignmentSquared, const float rCohesionSquared,
    const float weightDistancing, const float weightAlignment, const float weightCohesion
);

// Fonction d'encapsulation pour appeler le kernel
void updateBoidsCUDA(
    float* positionsX, float* positionsY, float* orientations, Types::Interaction* interactions,
    const int numBoids, const float envWidth, const float envHeight, const float speed, const float angVelocity, const float timeStep,
    const float halvedFov, const float rDistancingSquared, const float rAlignmentSquared, const float rCohesionSquared,
    const float weightDistancing, const float weightAlignment, const float weightCohesion
);

#endif // GPU_UTILS_CUH

#include "../include/gpu_utils.cuh"
#include <cuda_runtime.h>
#include <omp.h>

#define TWO_PIf (2.0f * M_PIf)

__device__ float* d_positionsX;
__device__ float* d_positionsY;
__device__ float* d_orientations;
__device__ Types::Interaction* d_interactions;

// Alloue la mémoire GPU pour les données des Boids
//extern "C" void allocateBoidDataOnGPU(Types::BoidData& boids) {
void allocateBoidDataOnGPU(Types::BoidData& boids) {
    auto dataSize = boids.positionsX.size() * sizeof(float);
    size_t interactionSize = boids.positionsX.size() * sizeof(Types::Interaction);
    //float* tempPointer;
    //Types::Interaction* tempInteractions;

    // Allocation pour chaque vecteur
    cudaMalloc(&boids.d_positionsX, dataSize);
    //cudaMemcpyToSymbol(d_positionsX, &tempPointer, sizeof(float*));
    cudaMalloc(&boids.d_positionsY, dataSize);
    //cudaMemcpyToSymbol(d_positionsY, &tempPointer, sizeof(float*));
    cudaMalloc(&boids.d_orientations, dataSize);
    //cudaMemcpyToSymbol(d_orientations, &tempPointer, sizeof(float*));
    cudaMalloc(&boids.d_interations, interactionSize);
    //cudaMemcpyToSymbol(d_interactions, &tempInteractions, sizeof(float*));
}

// Libère la mémoire GPU
void freeBoidDataOnGPU(Types::BoidData& boids) {
    cudaFree(boids.d_positionsX);
    cudaFree(boids.d_positionsY);
    cudaFree(boids.d_orientations);
    cudaFree(boids.d_interations);
}

// Transfère les données CPU -> GPU
void copyBoidDataToGPU(const Types::BoidData& boids) {
    size_t dataSize = boids.positionsX.size() * sizeof(float);
    size_t interactionSize = boids.positionsX.size() * sizeof(Types::Interaction);

    cudaMemcpy(boids.d_positionsX, boids.positionsX.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(boids.d_positionsY, boids.positionsY.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(boids.d_orientations, boids.orientations.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(boids.d_interations, boids.interactions.data(), interactionSize, cudaMemcpyHostToDevice);
}

// Transfère les données GPU -> CPU
void copyBoidDataToCPU(Types::BoidData& boids) {
    auto dataSize = boids.positionsX.size() * sizeof(float);
    size_t interactionSize = boids.positionsX.size() * sizeof(Types::Interaction);

    cudaMemcpy(boids.positionsX.data(), boids.d_positionsX, dataSize, cudaMemcpyDeviceToHost);
    cudaMemcpy(boids.positionsY.data(), boids.d_positionsY, dataSize, cudaMemcpyDeviceToHost);
    cudaMemcpy(boids.orientations.data(), boids.d_orientations, dataSize, cudaMemcpyDeviceToHost);
    cudaMemcpy(boids.interactions.data(), boids.d_interations, interactionSize, cudaMemcpyDeviceToHost);
}

// Réallocation si la taille change
void reallocateIfNecessary(Types::BoidData& boids) {
    // Libérer l'ancienne mémoire
    freeBoidDataOnGPU(boids);

    // Allouer de la nouvelle mémoire GPU
    allocateBoidDataOnGPU(boids);
}

// kernel
__global__ void updateBoidsKernel(
    float* positionsX, float* positionsY, float* orientations, Types::Interaction* interactions,
    const int numBoids, const int envWidth, const int envHeight, const float speed, const float angVelocity, const float timeStep,
    const float halvedFov, const float rDistancingSquared, const float rAlignmentSquared, const float rCohesionSquared,
    const float weightDistancing, const float weightAlignment, const float weightCohesion
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBoids) return;
    
    const int blockSize = blockDim.x;
    extern __shared__ float sharedMemory[];
    float* sharedPositionsX = sharedMemory;
    float* sharedPositionsY = &sharedPositionsX[blockSize];
    float* sharedOrientations = &sharedPositionsY[blockSize];

    float posX = positionsX[idx];
    float posY = positionsY[idx];
    float theta = orientations[idx];

    float alignX = 0.0f, alignY = 0.0f;
    float cohesionX = 0.0f, cohesionY = 0.0f;
    float distX = 0.0f, distY = 0.0f;
    int alignCount = 0, cohesionCount = 0, distCount = 0;

    for (int tile = 0; tile < (numBoids + blockSize - 1) / blockSize; ++tile) {
        int localIdx = threadIdx.x;
        int boidIdx = tile * blockSize + localIdx;

        if (boidIdx < numBoids) {
            sharedPositionsX[localIdx] = positionsX[boidIdx];
            sharedPositionsY[localIdx] = positionsY[boidIdx];
            sharedOrientations[localIdx] = orientations[boidIdx];
        }

        __syncthreads();

        for (int j = 0; j < blockSize && (tile * blockSize + j) < numBoids; ++j) {
            if (tile * blockSize + j == idx) continue;

            float dx = sharedPositionsX[j] - posX;
            float dy = sharedPositionsY[j] - posY;

            if (fabsf(dx) > 0.5f * envWidth) dx -= copysignf(envWidth, dx);
            if (fabsf(dy) > 0.5f * envHeight) dy -= copysignf(envHeight, dy);

            float distanceSquared = (dx * dx) + (dy * dy);

            if (distanceSquared > rCohesionSquared) continue;

            float angleToNeighbor = atan2f(dy, dx);
            float angleDifference = angleToNeighbor - theta;
            if (angleDifference > M_PIf) angleDifference -= TWO_PIf;
            else if (angleDifference < -M_PIf) angleDifference += TWO_PIf;

            bool isWithinFOV = fabsf(angleDifference) <= (halvedFov);

            if (!isWithinFOV) continue;
            
            if (distanceSquared < rDistancingSquared) {
                distX -= dx;
                distY -= dy;
                distCount++;
            } else if (distanceSquared < rAlignmentSquared) {
                alignX += __cosf(sharedOrientations[j]);
                alignY += __sinf(sharedOrientations[j]);
                alignCount++;
            } else if (distanceSquared < rCohesionSquared) {
                cohesionX += dx;
                cohesionY += dy;
                cohesionCount++;
            }
        }
        __syncthreads();
    }

    interactions[idx] = Types::Interaction::NONE;
    if (cohesionCount > 0) { cohesionX /= cohesionCount; cohesionY /= cohesionCount; interactions[idx] = Types::Interaction::COHESION; }
    if (alignCount > 0) { alignX /= alignCount; alignY /= alignCount; interactions[idx] = Types::Interaction::ALIGNMENT; }
    if (distCount > 0) { distX /= distCount; distY /= distCount; interactions[idx] = Types::Interaction::DISTANCING; }

    if (alignCount != 0 || cohesionCount != 0 || distCount != 0) {
        float newDirX = weightDistancing * distX + weightAlignment * alignX + weightCohesion * cohesionX;
        float newDirY = weightDistancing * distY + weightAlignment * alignY + weightCohesion * cohesionY;
        float newOrientation = atan2f(newDirY, newDirX);

        float angleDifference = newOrientation - theta;
        if (angleDifference > M_PIf) angleDifference -= TWO_PIf;
        else if (angleDifference < -M_PIf) angleDifference += TWO_PIf;

        float angularChange = fminf(fmaxf(angleDifference, -angVelocity * timeStep), angVelocity * timeStep);
        theta += angularChange;
        if (theta > M_PIf) theta -= TWO_PIf;
        else if (theta < -M_PIf) theta += TWO_PIf;
    }
    
    posX += speed * cosf(theta) * timeStep;
    posY += speed * sinf(theta) * timeStep;

    if (posX < 0) posX += envWidth;
    else if (posX >= envWidth) posX -= envWidth;
    if (posY < 0) posY += envHeight;
    else if (posY >= envHeight) posY -= envHeight;

    positionsX[idx] = posX;
    positionsY[idx] = posY;
    orientations[idx] = theta;
}

// Fonction d'encapsulation pour appeler le kernel
void updateBoidsCUDA(
    float* positionsX, float* positionsY, float* orientations, Types::Interaction* interactions,
    const int numBoids, const int envWidth, const int envHeight, const float speed, const float angVelocity, const float timeStep,
    const float halvedFov, const float rDistancingSquared, const float rAlignmentSquared, const float rCohesionSquared,
    const float weightDistancing, const float weightAlignment, const float weightCohesion
) {
    // Définir le nombre de threads par bloc et de blocs
    int threadsPerBlock = 256;
    int blocksPerGrid = (numBoids + threadsPerBlock - 1) / threadsPerBlock;
size_t sharedMemSize = 3 * threadsPerBlock * sizeof(float); // Taille de la mémoire partagée
    // Appeler le kernel
    updateBoidsKernel<<<blocksPerGrid, threadsPerBlock, sharedMemSize>>>(
        positionsX, positionsY, orientations, interactions,
        numBoids, envWidth, envHeight, speed, angVelocity, timeStep,
        halvedFov, rDistancingSquared, rAlignmentSquared, rCohesionSquared,
        weightDistancing, weightAlignment, weightCohesion
    );

    // Synchroniser pour s'assurer que le kernel est terminé
    cudaDeviceSynchronize();
}
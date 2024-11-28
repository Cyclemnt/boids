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
    //float* tempPointer;
    //Types::Interaction* tempInteractions;

    // Allocation pour chaque vecteur
    cudaMalloc(&boids.d_positionsX, dataSize);
    //cudaMemcpyToSymbol(d_positionsX, &tempPointer, sizeof(float*));
    cudaMalloc(&boids.d_positionsY, dataSize);
    //cudaMemcpyToSymbol(d_positionsY, &tempPointer, sizeof(float*));
    cudaMalloc(&boids.d_orientations, dataSize);
    //cudaMemcpyToSymbol(d_orientations, &tempPointer, sizeof(float*));
    cudaMalloc(&boids.d_interations, dataSize);
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
    
    // Taille du bloc et mémoire partagée
    const int blockSize = blockDim.x;
    extern __shared__ float sharedMemory[];

    // Diviser la mémoire partagée
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

    // Charger les boids en mémoire partagée en boucle
    for (int tile = 0; tile < (numBoids + blockSize - 1) / blockSize; ++tile) {
        // Charger une "tuile" de boids dans la mémoire partagée
        int localIdx = threadIdx.x;
        int boidIdx = tile * blockSize + localIdx;

        if (boidIdx < numBoids) {
            sharedPositionsX[localIdx] = positionsX[boidIdx];
            sharedPositionsY[localIdx] = positionsY[boidIdx];
            sharedOrientations[localIdx] = orientations[boidIdx];
        }

        __syncthreads(); // Synchronisation après le chargement des données


        // Calculer les interactions avec les boids dans cette tuile
        for (int j = 0; j < blockSize && (tile * blockSize + j) < numBoids; ++j) {
            if (j == idx) continue; // Ne pas traiter le Boid en question

            // Calculer la distance
            float dx = sharedPositionsX[j] - posX;
            float dy = sharedPositionsY[j] - posY;
            
            // Calculer la distance torique
            if (dx > 0.5f * envWidth) dx = envWidth - dx;
            if (dy > 0.5f * envHeight) dy = envHeight - dy;

            // Calculer la distance euclidienne avec les distances minimales en x et y
            float distanceSquared = (dx * dx) + (dy * dy);

            if (distanceSquared > rCohesionSquared) continue;

            // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe x
            float angleToNeighbor = atan2f(dy, dx);
            // Calculer la différence angulaire par rapport à l'orientation du boid
            float angleDifference = angleToNeighbor - theta;
            if (angleDifference > M_PIf) angleDifference -= TWO_PIf;
            else if (angleDifference < -M_PIf) angleDifference += TWO_PIf;


            bool isWithinFOV = fabsf(angleDifference) <= (halvedFov);

            if (!isWithinFOV) continue;
            
            // Règle 3 : Cohésion
            if (distanceSquared < rCohesionSquared) {
                cohesionX += dx;
                cohesionY += dy;
                cohesionCount++;
                // Règle 2 : Alignement
                if (distanceSquared < rAlignmentSquared) {
                    alignX += __cosf(sharedOrientations[j]);
                    alignY += __sinf(sharedOrientations[j]);
                    alignCount++;
                    // Règle 1 : Distanciation
                    if (distanceSquared < rDistancingSquared) {
                        distX -= dx;
                        distY -= dy;
                        distCount++;
                    }
                }
            }
        }
        __syncthreads(); // Synchronisation avant de passer à la prochaine tuile
    }

    // Moyenne des vecteurs
    interactions[idx] = Types::Interaction::NONE;
    if (cohesionCount > 0) { cohesionX /= cohesionCount; cohesionY /= cohesionCount; interactions[idx] = Types::Interaction::COHESION; }
    if (alignCount > 0) { alignX /= alignCount; alignY /= alignCount; interactions[idx] = Types::Interaction::ALIGNMENT; }
    if (distCount > 0) { distX /= distCount; distY /= distCount; interactions[idx] = Types::Interaction::DISTANCING; }

    
    if (alignCount != 0 || cohesionCount != 0 || distCount != 0) {
        // Combiner les vecteurs
        float newDirX = weightDistancing * distX + weightAlignment * alignX + weightCohesion * cohesionX;
        float newDirY = weightDistancing * distY + weightAlignment * alignY + weightCohesion * cohesionY;

        // Calculer la nouvelle orientation
        float newOrientation = atan2f(newDirY, newDirX);
        // Normaliser les angles entre -π et π
        float angleDifference = newOrientation - theta;
        if (angleDifference > M_PIf) angleDifference -= TWO_PIf;
        else if (angleDifference < -M_PIf) angleDifference += TWO_PIf;
        // Limiter la vitesse angulaire
        float angularChange = fminf(fmaxf(angleDifference, -angVelocity * timeStep), angVelocity * timeStep); // std::clamp
        // Mettre à jour l'orientation
        theta += angularChange;
        if (theta > M_PIf) theta -= TWO_PIf;
        else if (theta < -M_PIf) theta += TWO_PIf;
    }
    
    // Calculer la nouvelle position
    posX += speed * cosf(theta) * timeStep;
    posY += speed * sinf(theta) * timeStep;
    // Assurer le comportement torique de l'environnement
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

    // Taille mémoire partagée
    size_t sharedMemorySize = 3 * threadsPerBlock * sizeof(float);

    // Appeler le kernel
    updateBoidsKernel<<<blocksPerGrid, threadsPerBlock, sharedMemorySize>>>(
        positionsX, positionsY, orientations, interactions,
        numBoids, envWidth, envHeight, speed, angVelocity, timeStep,
        halvedFov, rDistancingSquared, rAlignmentSquared, rCohesionSquared,
        weightDistancing, weightAlignment, weightCohesion
    );

    // Synchroniser pour s'assurer que le kernel est terminé
    cudaDeviceSynchronize();
}
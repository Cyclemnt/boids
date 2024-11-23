#include "../include/gpu_utils.cuh"
#include <cuda_runtime.h>

// Alloue la mémoire GPU pour les données des Boids
void allocateBoidDataOnGPU(Types::BoidData& boids) {
    auto dataSize = boids.positionsX.size() * sizeof(float);

    // Allocation pour chaque vecteur
    cudaMalloc(&boids.d_positionsX, dataSize);
    cudaMalloc(&boids.d_positionsY, dataSize);
    cudaMalloc(&boids.d_orientations, dataSize);
    cudaMalloc(&boids.d_interations, dataSize);
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
    auto dataSize = boids.positionsX.size() * sizeof(float);

    cudaMemcpy(boids.d_positionsX, boids.positionsX.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(boids.d_positionsY, boids.positionsY.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(boids.d_orientations, boids.orientations.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(boids.d_interations, boids.interactions.data(), dataSize, cudaMemcpyHostToDevice);
}

// Transfère les données GPU -> CPU
void copyBoidDataToCPU(Types::BoidData& boids) {
    auto dataSize = boids.positionsX.size() * sizeof(float);

    cudaMemcpy(boids.positionsX.data(), boids.d_positionsX, dataSize, cudaMemcpyDeviceToHost);
    cudaMemcpy(boids.positionsY.data(), boids.d_positionsY, dataSize, cudaMemcpyDeviceToHost);
    cudaMemcpy(boids.orientations.data(), boids.d_orientations, dataSize, cudaMemcpyDeviceToHost);
    cudaMemcpy(boids.interactions.data(), boids.d_interations, dataSize, cudaMemcpyDeviceToHost);
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
    const int numBoids, const float envWidth, const float envHeight, const float speed, const float angVelocity, const float timeStep,
    const float halvedFov, const float rDistancingSquared, const float rAlignmentSquared, const float rCohesionSquared,
    const float weightDistancing, const float weightAlignment, const float weightCohesion
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBoids) return;

    const float two_pi = 2 * M_PI;

    
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

            float absDx = fabsf(dx);
            float absDy = fabsf(dy);

            // Calculer la distance torique
            if (absDx > 0.5 * envWidth) absDx = envWidth - absDx;
            if (absDy > 0.5 * envHeight) absDy = envHeight - absDy;

            // Calculer la distance euclidienne avec les distances minimales en x et y
            float distanceSquared = (absDx * absDx) + (absDy * absDy);

            // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe x
            float angleToNeighbor = atan2f(dy, dx);
            // Calculer la différence angulaire par rapport à l'orientation du boid
            float angleDifference = fmodf(fmodf(angleToNeighbor - theta + M_PI, two_pi) + two_pi, two_pi) - M_PI;
            bool isWithinFOV = fabsf(angleDifference) <= (halvedFov);

            if (!isWithinFOV) continue;

            // Règle 1 : Distanciation
            if (distanceSquared < rDistancingSquared) {
                distX -= dx;
                distY -= dy;
                distCount++;
            }
            // Règle 2 : Alignement
            if (distanceSquared < rAlignmentSquared) {
                alignX += cosf(sharedOrientations[j]);
                alignY += sinf(sharedOrientations[j]);
                alignCount++;
            }
            // Règle 3 : Cohésion
            if (distanceSquared < rCohesionSquared) {
                cohesionX += dx;
                cohesionY += dy;
                cohesionCount++;
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
        float angleDifference = fmodf(fmodf(newOrientation - theta + M_PI, two_pi) + two_pi, two_pi) - M_PI;
        // Limiter la vitesse angulaire
        float angularChange = fminf(fmaxf(angleDifference, -angVelocity * timeStep), angVelocity * timeStep); // std::clamp
        // Mettre à jour l'orientation
        theta += angularChange;
        theta = fmodf(fmodf(theta, two_pi) + two_pi, two_pi); // S'assurer que theta est dans [0, 2π[
    }
    
    // Calculer la nouvelle position
    posX += speed * cosf(theta) * timeStep;
    posY += speed * sinf(theta) * timeStep;
    // Assurer le comportement torique de l'environnement
    if (posX < 0) { posX += envWidth; }
    else if (posX >= envWidth) { posX -= envWidth; }
    if (posY < 0) { posY += envHeight; }
    else if (posY >= envHeight) { posY -= envHeight; }

    positionsX[idx] = posX;
    positionsY[idx] = posY;
    orientations[idx] = theta;
}

// Fonction d'encapsulation pour appeler le kernel
void updateBoidsCUDA(
    float* positionsX, float* positionsY, float* orientations, Types::Interaction* interactions,
    const int numBoids, const float envWidth, const float envHeight, const float speed, const float angVelocity, const float timeStep,
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
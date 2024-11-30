#include "../include/gpu_utils.cuh"
#include "../include/constants.hpp"
#include <cuda_runtime.h>
#include <omp.h>

// Kernel
__global__ void updateBoidsKernel(float* x, float* y, float* theta, int* interactions, const int numBoids) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBoids) return;

    float posX = x[idx];
    float posY = y[idx];
    float angle = theta[idx];

    float alignX = 0.0f, alignY = 0.0f;
    float cohesionX = 0.0f, cohesionY = 0.0f;
    float distX = 0.0f, distY = 0.0f;
    int alignCount = 0, cohesionCount = 0, distCount = 0;

    // Calculer les interactions avec les boids dans cette tuile
    for (int j = 0; j < numBoids; ++j) {
        if (j == idx) continue; // Ne pas traiter le Boid en question

        // Calculer la distance
        float dx = x[j] - posX;
        float dy = y[j] - posY;
        
        // Calculer la distance torique
        if (dx > HALVED_ENV_WIDTH) dx -= ENV_WIDTH;
        else if (dx < -HALVED_ENV_WIDTH) dx += ENV_WIDTH;
        if (dy > HALVED_ENV_HEIGHT) dy -= ENV_HEIGHT;
        else if (dy < -HALVED_ENV_HEIGHT) dy += ENV_HEIGHT;

        // Calculer la distance euclidienne avec les distances minimales en x et y
        float distanceSquared = (dx * dx) + (dy * dy);

        if (distanceSquared > R_COHESION_SQUARED) continue; // éviter de faire les calculs suivants pour rien

        // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe x
        float angleToNeighbor = atan2f(dy, dx);
        // Calculer la différence angulaire par rapport à l'orientation du boid
        float angleDifference = angleToNeighbor - angle;
        if (angleDifference > M_PIf) angleDifference -= TWO_PIf;
        else if (angleDifference < -M_PIf) angleDifference += TWO_PIf;


        bool isWithinFOV = fabsf(angleDifference) <= (HALVED_FOV);

        if (!isWithinFOV) continue;
        
        // Règle 1 : Distanciation
        if (distanceSquared < R_DISTANCING_SQUARED) {
            distX -= dx;
            distY -= dy;
            distCount++;
        }
        // Règle 2 : Alignement
        else if (distanceSquared < R_ALIGNMENT_SQUARED) {
            alignX += __cosf(theta[j]);
            alignY += __sinf(theta[j]);
            alignCount++;
        }
        // Règle 3 : Cohésion
        else if (distanceSquared < R_COHESION_SQUARED) {
            cohesionX += dx;
            cohesionY += dy;
            cohesionCount++;
        }
    }

    // Moyenne des vecteurs
    interactions[idx] = 0;
    if (cohesionCount > 0) { cohesionX /= cohesionCount; cohesionY /= cohesionCount; interactions[idx] = 3; }
    if (alignCount > 0) { alignX /= alignCount; alignY /= alignCount; interactions[idx] = 2; }
    if (distCount > 0) { distX /= distCount; distY /= distCount; interactions[idx] = 1; }

    
    if (alignCount != 0 || cohesionCount != 0 || distCount != 0) {
        // Combiner les vecteurs
        float newDirX = WEIGHT_DISTANCING * distX + WEIGHT_ALIGNMENT * alignX + WEIGHT_COHESION * cohesionX;
        float newDirY = WEIGHT_DISTANCING * distY + WEIGHT_ALIGNMENT * alignY + WEIGHT_COHESION * cohesionY;

        // Calculer la nouvelle orientation
        float newOrientation = atan2f(newDirY, newDirX);
        // Normaliser les angles entre -π et π
        float angleDifference = newOrientation - angle;
        if (angleDifference > M_PIf) angleDifference -= TWO_PIf;
        else if (angleDifference < -M_PIf) angleDifference += TWO_PIf;
        // Limiter la vitesse angulaire
        float angularChange = fminf(fmaxf(angleDifference, -ANG_V * TIME_STEP), ANG_V * TIME_STEP); // std::clamp
        // Mettre à jour l'orientation
        angle += angularChange;
        if (angle > M_PIf) angle -= TWO_PIf;
        else if (angle < -M_PIf) angle += TWO_PIf;
    }
    
    // Calculer la nouvelle position
    posX += SPEED * __cosf(angle) * TIME_STEP;
    posY += SPEED * __sinf(angle) * TIME_STEP;
    // Assurer le comportement torique de l'environnement
    if (posX < 0) posX += ENV_WIDTH;
    else if (posX >= ENV_WIDTH) posX -= ENV_WIDTH;
    if (posY < 0) posY += ENV_HEIGHT;
    else if (posY >= ENV_HEIGHT) posY -= ENV_HEIGHT;

    x[idx] = posX;
    y[idx] = posY;
    theta[idx] = angle;
}

// Fonction d'encapsulation pour appeler le kernel
void updateBoidsCUDA(float* x, float* y, float* theta, int* interactions, const int numBoids) {
    // Définir le nombre de threads par bloc et de blocs
    int threadsPerBlock = 256;
    int blocksPerGrid = (numBoids + threadsPerBlock - 1) / threadsPerBlock;
    //size_t sharedMemSize = 3 * threadsPerBlock * sizeof(float); // Pour x, y et theta
    // Appeler le kernel
    updateBoidsKernel<<<blocksPerGrid, threadsPerBlock>>>(x, y, theta, interactions, numBoids);

    // Synchroniser pour s'assurer que le kernel est terminé
    cudaDeviceSynchronize();
}
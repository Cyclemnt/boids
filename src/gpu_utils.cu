#include "../include/gpu_utils.cuh"
#include "../include/constants.hpp"
#include <cuda_runtime.h>
#include <thrust/device_ptr.h> // Pour partial sum
#include <thrust/scan.h> // Pour partial sum

// Kernel
__global__ void updateBoidsKernel(float* x, float* y, float* theta, int* interaction, int* cellCount, int* particleMap, const int numBoids, const int numCellsWidth, const int numCellsHeight, const float inverseCellWidth, const float inverseCellHeight) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBoids) return;

    // Définir les constantes avec constexpr
    constexpr int rDistancingSquared = R_DISTANCING * R_DISTANCING;
    constexpr int rAlignmentSquared = R_ALIGNMENT * R_ALIGNMENT;
    constexpr int rCohesionSquared = R_COHESION * R_COHESION;
    constexpr int halvedEnvWidth = ENV_WIDTH / 2.0f;
    constexpr int halvedEnvHeight = ENV_HEIGHT / 2.0f;
    constexpr float halvedFOV = FOV / 2.0f;
    constexpr float twoPif = 2.0f * M_PIf;

    // Récupère les infos du boid courant
    float posX = x[idx];
    float posY = y[idx];
    float angle = theta[idx];

    int cellX = floor(posX * inverseCellWidth);
    int cellY = floor(posY * inverseCellHeight);

    float alignX = 0.0f, alignY = 0.0f;
    float cohesionX = 0.0f, cohesionY = 0.0f;
    float distX = 0.0f, distY = 0.0f;
    int alignCount = 0, cohesionCount = 0, distCount = 0;

    // Parcourir les cellules voisines (3x3 voisinage)
    for (int offsetY = -1; offsetY <= 1; ++offsetY) {
        for (int offsetX = -1; offsetX <= 1; ++offsetX) {
            int neighborX = cellX + offsetX;
            int neighborY = cellY + offsetY;

            if (neighborX < 0) neighborX += numCellsWidth;
            if (neighborX >= numCellsWidth) neighborX -= numCellsWidth;
            if (neighborY < 0) neighborY += numCellsHeight;
            if (neighborY >= numCellsHeight) neighborY -= numCellsHeight;

            int neighborIndex = neighborX + neighborY * numCellsWidth;

            // Parcourir les boids dans cette cellule
            for (int i = cellCount[neighborIndex]; i < cellCount[neighborIndex + 1]; ++i) {
                int neighborBoidIdx = particleMap[i];

                // Évite le boid courant
                if (neighborBoidIdx == idx) continue;

                // Calculer la distance
                float dx = x[neighborBoidIdx] - posX;
                float dy = y[neighborBoidIdx] - posY;
                
                // Calculer la distance torique
                if (dx > halvedEnvWidth) dx -= ENV_WIDTH;
                else if (dx < -halvedEnvWidth) dx += ENV_WIDTH;
                if (dy > halvedEnvHeight) dy -= ENV_HEIGHT;
                else if (dy < -halvedEnvHeight) dy += ENV_HEIGHT;

                // Calculer la distance euclidienne avec les distances minimales en x et y
                float distanceSquared = (dx * dx) + (dy * dy);

                if (distanceSquared > rCohesionSquared) continue; // éviter de faire les calculs suivants pour rien

                // Calculer l'angle du vecteur (dx, dy) par rapport à l'axe x
                float angleToNeighbor = atan2f(dy, dx);
                // Calculer la différence angulaire par rapport à l'orientation du boid
                float angleDifference = angleToNeighbor - angle;
                if (angleDifference > M_PIf) angleDifference -= twoPif;
                else if (angleDifference < -M_PIf) angleDifference += twoPif;


                bool isWithinFOV = fabsf(angleDifference) <= (halvedFOV);

                if (!isWithinFOV) continue;
                
                // Règle 1 : Distanciation
                if (distanceSquared < rDistancingSquared) {
                    distX -= dx;
                    distY -= dy;
                    distCount++;
                }
                // Règle 2 : Alignement
                else if (distanceSquared < rAlignmentSquared) {
                    alignX += __cosf(theta[neighborBoidIdx]);
                    alignY += __sinf(theta[neighborBoidIdx]);
                    alignCount++;
                }
                // Règle 3 : Cohésion
                else if (distanceSquared < rCohesionSquared) {
                    cohesionX += dx;
                    cohesionY += dy;
                    cohesionCount++;
                }
            }
        }
    }

    // Moyenne des vecteurs
    interaction[idx] = 0;
    if (cohesionCount > 0) { cohesionX /= cohesionCount; cohesionY /= cohesionCount; interaction[idx] = 3; }
    if (alignCount > 0) { alignX /= alignCount; alignY /= alignCount; interaction[idx] = 2; }
    if (distCount > 0) { distX /= distCount; distY /= distCount; interaction[idx] = 1; }

    
    if (alignCount != 0 || cohesionCount != 0 || distCount != 0) {
        // Combiner les vecteurs
        float newDirX = WEIGHT_DISTANCING * distX + WEIGHT_ALIGNMENT * alignX + WEIGHT_COHESION * cohesionX;
        float newDirY = WEIGHT_DISTANCING * distY + WEIGHT_ALIGNMENT * alignY + WEIGHT_COHESION * cohesionY;

        // Calculer la nouvelle orientation
        float newOrientation = atan2f(newDirY, newDirX);
        // Normaliser les angles entre -π et π
        float angleDifference = newOrientation - angle;
        if (angleDifference > M_PIf) angleDifference -= twoPif;
        else if (angleDifference < -M_PIf) angleDifference += twoPif;
        // Limiter la vitesse angulaire
        float angularChange = fminf(fmaxf(angleDifference, -ANG_V * TIME_STEP), ANG_V * TIME_STEP); // std::clamp
        // Mettre à jour l'orientation
        angle += angularChange;
        if (angle > M_PIf) angle -= twoPif;
        else if (angle < -M_PIf) angle += twoPif;
    }
    
    // Calculer la nouvelle position
    posX += SPEED * __cosf(angle) * TIME_STEP;
    posY += SPEED * __sinf(angle) * TIME_STEP;
    // Assurer le comportement torique de l'environnement
    if (posX < 0) posX += ENV_WIDTH;
    if (posX >= ENV_WIDTH) posX -= ENV_WIDTH;
    if (posY < 0) posY += ENV_HEIGHT;
    if (posY >= ENV_HEIGHT) posY -= ENV_HEIGHT;

    x[idx] = posX;
    y[idx] = posY;
    theta[idx] = angle;
}

// Fonction d'encapsulation pour appeler le kernel
void updateBoidsCUDA(
    float* d_x, float* d_y, float* d_theta, int* d_interaction, const int numBoids,
    int* d_cellCount, int* d_particleMap,
    const int numCells, const int numCellsWidth, const int numcellsHeight, const float inverseCellWidth, const float inverseCellHeight,
    unsigned char* d_image) {
    // Définir le nombre de threads par bloc et de blocs
    int threadsPerBlock = 256;
    int blocksPerGrid = (numBoids + threadsPerBlock - 1) / threadsPerBlock;

    // Compter les boids
    cudaMemset(d_cellCount, 0, (numCells + 1) * sizeof(int));
    fillCellCount<<<blocksPerGrid, threadsPerBlock>>>(d_x, d_y, d_cellCount, numBoids, numCellsWidth, numCells, inverseCellWidth, inverseCellHeight);
    cudaDeviceSynchronize();

    // Calcul des sommes partielles
    thrust::device_ptr<int> dev_ptr(d_cellCount);
    thrust::inclusive_scan(dev_ptr, dev_ptr + numCells + 1, dev_ptr);

    // Remplir particleMap
    fillParticleMap<<<blocksPerGrid, threadsPerBlock>>>(d_x, d_y, d_cellCount, d_particleMap, numBoids, numCellsWidth, inverseCellWidth, inverseCellHeight);
    cudaDeviceSynchronize();

    // Appeler le kernel principal
    updateBoidsKernel<<<blocksPerGrid, threadsPerBlock>>>(d_x, d_y, d_theta, d_interaction, d_cellCount, d_particleMap, numBoids, numCellsWidth, numcellsHeight, inverseCellWidth, inverseCellHeight);
    cudaDeviceSynchronize();

    // Remplir le buffer d'image après l'update
    cudaMemset(d_image, 0, ENV_WIDTH * ENV_HEIGHT * 3); // Nettoyage de l'image
    renderBoidsKernel<<<blocksPerGrid, threadsPerBlock>>>(d_x, d_y, d_interaction, d_image, numBoids, ENV_WIDTH);
    cudaDeviceSynchronize();
}

// Compter les boids par cellule
__global__ void fillCellCount(float* x, float* y, int* cellCount, const int numBoids, const int numCellsWidth, const int numCells, const float inverseCellWidth, const float inverseCellHeight) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBoids) return;

    int cellX = floor(x[idx] * inverseCellWidth);
    int cellY = floor(y[idx] * inverseCellHeight);
    int cellIndex = cellX + cellY * numCellsWidth;

    atomicAdd(&cellCount[cellIndex], 1);
}

// Remplir particleMap
__global__ void fillParticleMap(float* x, float* y, int* cellCount, int* particleMap, const int numBoids, const int numCellsWidth, const float inverseCellWidth, const float inverseCellHeight) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBoids) return;

    int cellX = floor(x[idx] * inverseCellWidth);
    int cellY = floor(y[idx] * inverseCellHeight);
    int cellIndex = cellX + cellY * numCellsWidth;

    // Atomic decrement to fill particleMap in correct order
    int mapIndex = atomicAdd(&cellCount[cellIndex], -1) - 1;
    particleMap[mapIndex] = idx;
}

__global__ void renderBoidsKernel(float* x, float* y, int* interaction, unsigned char* image, int numBoids, int envWidth) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= numBoids) return;

    // Déterminer la couleur selon l'interaction
    unsigned char r, g, b;
    switch (interaction[idx]) {
        case 1: r = 255; g = 0;   b = 0;   break;  // Rouge
        case 2: r = 0;   g = 255; b = 0;   break;  // Vert
        case 3: r = 0;   g = 0;   b = 255; break;  // Bleu
        default: r = 127; g = 127; b = 0; break;   // Jaune
    }

    // Position du boid
    int pixelX = static_cast<int>(x[idx]);
    int pixelY = static_cast<int>(y[idx]);

    int pixelIndex = (pixelY * envWidth + pixelX) * 3;
    image[pixelIndex]     = b;  // Blue
    image[pixelIndex + 1] = g;  // Green
    image[pixelIndex + 2] = r;  // Red
}

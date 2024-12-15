#ifndef GPU_UTILS_CUH
#define GPU_UTILS_CUH

#include <cuda_runtime.h>

// Fonction d'encapsulation pour appeler le kernel
void updateBoidsCUDA(
    float* d_x, float* d_y, float* d_theta, unsigned char* d_image, const int numBoids,
    int* d_cellCount, int* d_boidMap,
    const int numCells, const int numCellsWidth, const int numcellsHeight, const float inverseCellWidth, const float inverseCellHeight
);

// Kernel principal
__global__ void updateBoidsKernel(float* x, float* y, float* theta, unsigned char* image, int* cellCount, int* boidMap, const int numBoids, const int numCellsWidth, const int numCellsHeight, const float inverseCellWidth, const float inverseCellHeight);
// Compter les boids par cellule
__global__ void fillCellCount(float* x, float* y, int* cellCount, const int numBoids, const int numCellsWidth, const float inverseCellWidth, const float inverseCellHeight);
// Remplir boidMap
__global__ void fillBoidMap(float* x, float* y, int* cellCount, int* boidMap, const int numBoids, const int numCellsWidth, const float inverseCellWidth, const float inverseCellHeight);

#endif // GPU_UTILS_CUH

#ifndef GPU_UTILS_CUH
#define GPU_UTILS_CUH

#include <cuda_runtime.h>

// Kernel
__global__ void updateBoidsKernel(float* x, float* y, float* theta, int mouseX, int mouseY, unsigned char* image, int* cellCount, int* particleMap, const int numBoids, const int numCellsWidth, const int numCellsHeight, const float inverseCellWidth, const float inverseCellHeight);

// Fonction d'encapsulation pour appeler le kernel
void updateBoidsCUDA(
    float* d_x, float* d_y, float* d_theta, unsigned char* d_image, const int numBoids,
    int mouseX, int mouseY,
    int* d_cellCount, int* d_particleMap,
    const int numCells, const int numCellsWidth, const int numcellsHeight, const float inverseCellWidth, const float inverseCellHeight);

__global__ void fillCellCount(float* x, float* y, int* cellCount, const int numBoids, const int numCellsWidth, const int numCells, const float inverseCellWidth, const float inverseCellHeight);
__global__ void fillParticleMap(float* x, float* y, int* cellCount, int* particleMap, const int numBoids, const int numCellsWidth, const float inverseCellWidth, const float inverseCellHeight);

#endif // GPU_UTILS_CUH

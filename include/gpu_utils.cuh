#ifndef GPU_UTILS_CUH
#define GPU_UTILS_CUH

#include <cuda_runtime.h>

// Kernel
__global__ void updateBoidsKernel(float* x, float* y, float* theta, int* interactions, const int numBoids);

// Fonction d'encapsulation pour appeler le kernel
void updateBoidsCUDA(float* x, float* y, float* theta, int* interactions, const int numBoids);

#endif // GPU_UTILS_CUH

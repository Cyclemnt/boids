cmake_minimum_required(VERSION 3.3)
project(boids)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CUDA REQUIRED)

include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS} # Inclure les en-têtes CUDA
)

set(SOURCES
    src/main.cpp
    src/boid.cpp
    src/simulation.cpp
    src/zone.cpp
)

set(CUDA_SOURCES
    src/cuda_kernels.cu
)

set(HEADERS
    include/types.hpp
    include/boid.hpp
    include/simulation.hpp
    include/zone.hpp
    include/cuda_kernels.cuh
)

# Définir les flags NVCC pour la compilation CUDA
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=native")  # Détection automatique de l'architecture GPU


# Ajouter une cible CUDA pour les fichiers CUDA
cuda_add_library(cuda_kernels STATIC ${CUDA_SOURCES})

# Ajouter un exécutable pour le projet principal
add_executable(main ${SOURCES} ${HEADERS})

# Lier les bibliothèques nécessaires
target_link_libraries(main ${OpenCV_LIBS} OpenMP::OpenMP_CXX cuda_kernels ${CUDA_LIBRARIES})
cmake_minimum_required(VERSION 3.18)
project(boids LANGUAGES CXX CUDA)

# Spécifier le standard C++ requis
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Spécifier l'architecture CUDA (Ampere)
set(CMAKE_CUDA_ARCHITECTURES 86)  # Architecture Ampere, ajouter d'autres si nécessaire

# Trouver les packages requis
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Activer le support CUDA
enable_language(CUDA)

# Définir les répertoires d'inclusion
include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

# Définir les fichiers sources et headers
set(SOURCES
    src/main.cpp
    src/simulation.cpp
)

set(CUDA_SOURCES
    src/gpu_utils.cu
)

set(HEADERS
    include/constants.hpp
    include/simulation.hpp
    include/gpu_utils.cuh
)

# Ajouter une bibliothèque CUDA
add_library(cuda_kernels STATIC ${CUDA_SOURCES})

# Ajouter un exécutable principal
add_executable(main ${SOURCES})

# Lier les bibliothèques nécessaires
target_link_libraries(main PRIVATE
    ${OpenCV_LIBS} 
    cuda_kernels
)

# Activer CUDA pour l'exécutable principal
set_target_properties(main PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# Boids Simulation  

This project implements a simulation of boids, mobile entities following simple rules to reproduce collective behaviors. Built on a common core, the project is divided into three main branches, each exploring a different extension.  

<p align="center">
  <img src="demo.gif" alt="Demo"/>
</p>


## Project Description  
Boids are a classic example of emergent behavior: complexity arises from the interaction of individual agents (the boids) that follow a small set of simple rules.  
The core rules implemented are:  
1. **Separation** : steer to avoid crowding local flockmates  
2. **Alignment** : steer towards the average heading of local flockmates  
3. **Cohesion** : steer to move toward the average position (center of mass) of local flockmates  

## Features  
### Base Simulation  
- Real-time rendering of boids with distinct colors for each rule (separation, alignment, cohesion).  
- Toroidal environment (connected borders).  

### Extensions (different branches of the repository)  
#### Mouse Interaction (branch: [`extension-mouse`](https://github.com/Cyclemnt/boids/tree/extension-mouse))  
- Add or remove boids with mouse clicks.  
- Enable/disable mouse-following behavior.  

#### Performance Optimization (branch: [`extension-performance`](https://github.com/Cyclemnt/boids/tree/extension-performance))  
- Advanced optimization using CUDA and spatial hashing.
- Supports simulations with up to one million boids.  

#### Predation (branch: [`extension-predation`](https://github.com/Cyclemnt/boids/tree/extension-predation))  
- Adds predators and static resources (food).  
- Hunting and reproduction dynamics.  
- Entities can transform based on interactions (e.g., a captured boid becomes a predator).  

## Dependencies
- **[OpenCV](https://opencv.org/)** : for graphical rendering.
- **[OpenMP](https://www.openmp.org/)** : for CPU parallelization.
- **[CMake](https://cmake.org/)** : for configuration and build.
- **C++11** or later.
- **[CUDA](https://developer.nvidia.com/cuda-toolkit)** : (required only for the `extension-performance branch`)

## Installation & Build
**Clone the repository**:
   ```bash
   git clone https://github.com/Cyclemnt/boids.git
  ```
**Build and run**:
   ```bash
   cd boids
   mkdir build
   cd build
   cmake ..
   make
   ./main
  ```

## Contributors
- FERRIER Simon
- LAMOULLER Clément
- PARIZOT Luan

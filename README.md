# Simulation de Boids - Optimisation des Performances  

Cette branche du projet est dédiée à l’optimisation des performances de la simulation, permettant de gérer des populations de boids bien plus importantes grâce à des techniques avancées.  

## Description de l'extension  
L'optimisation repose sur deux approches principales :  
1. **Parallélisation via CUDA** : Utilisation du GPU pour effectuer les calculs massivement parallèles, déchargeant ainsi le CPU.  
2. **Spatial Hashing** : Découpage de l’espace en cellules, réduisant le nombre de comparaisons nécessaires pour chaque boid.  

Avec ces techniques, la simulation peut gérer jusqu’au million de boids tout en maintenant un affichage fluide.  

## Dépendances  
- **C++11** ou version ultérieure.  
- **[CMake](https://cmake.org/)** : Pour la configuration et la compilation.  
- **[OpenCV](https://opencv.org/)** : Pour l’affichage graphique.  
- **[CUDA](https://developer.nvidia.com/cuda-toolkit)** ainsi qu'un **GPU NVIDIA** : Pour la parallélisation GPU.  

## Installation et Compilation  
**Cloner le dépôt** :  
   ```bash
   git clone -b extension-performance https://github.com/Cyclemnt/boids.git
   ```  

**Compiler** :  
   ```bash
   cd boids
   mkdir build
   cd build
   cmake ..  
   make
   ./main
   ```  
```
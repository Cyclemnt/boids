# Simulation de Boids  

Ce projet propose une simulation de boids, des entités mobiles suivant des règles simples pour simuler des comportements collectifs. Basé sur une structure commune, il se décline en trois branches principales, chacune explorant une extension unique.  

## Description du projet  
Les boids sont un exemple de comportement émergent, c'est-à-dire que la complexité des boids provient de l'interaction d'agents individuels (les boids, dans ce cas) adhérant à un ensemble de règles simples. Les règles appliquées dans le monde des boids le plus simple sont les suivantes :  
1. **Distanciation** : orienter pour éviter d'entasser les membres du troupeau local  
2. **Alignement** : orienter vers le cap moyen des membres du troupeau local  
3. **Cohésion** : orienter pour se déplacer vers la position moyenne (centre de masse) des membres du troupeau local  

## Fonctionnalités  
### Simulation de base  
- Affichage en temps réel des boids avec des couleurs distinctes pour chaque règle (distanciation, alignement, cohésion).  
- Système d’environnement torique (bords connectés).  

### Extensions  
#### Interaction avec la souris  
- Ajout ou suppression de boids via clics de souris.  
- Fonction de suivi de la souris par les boids, activable/désactivable.  

#### Optimisation des performances  
- Optimisation avancée via CUDA et spatial hashing, permettant la simulation du million de boids.  

#### Prédation  
- Introduction de prédateurs et de ressources statiques (nourriture).  
- Dynamique de chasse et reproduction.  
- Transformation des entités selon des interactions spécifiques (ex : boid capturé devient prédateur).  

## Dépendances
- **[OpenCV](https://opencv.org/)** : Pour les affichages graphiques.
- **[OpenMP](https://www.openmp.org/)** : Pour la parallélisation CPU.
- **[CMake](https://cmake.org/)** : Pour la configuration et la construction du projet.
- **C++11** ou version ultérieure.
- **[CUDA](https://developer.nvidia.com/cuda-toolkit)** : Pour la parallélisation de la branche extension-performance

## Installation et Compilation
**Cloner le dépôt** :
   ```bash
   git clone https://github.com/Cyclemnt/boids.git
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

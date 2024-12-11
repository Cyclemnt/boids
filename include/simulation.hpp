#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>

class Simulation {
    private : 
        // Poses
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> theta;
        // Spatial hashing
        std::vector<int> cellCount; // Nombre de boids par cellule (initialement), devient la somme partielle
        std::vector<int> particleMap; // Indices des boids triés par cellule
        // Pointeurs GPU
        float* d_x;
        float* d_y;
        float* d_theta;
        unsigned char* d_image;
        int* d_cellCount;
        int* d_particleMap;
        // Divers
        bool running, paused; // état de la simulation
        const float inverseCellWidth, inverseCellHeight; // Inverse des dimensions de cellule
        const int numCells, numCellWidth, numCellHeight; // Nombre de cellules, en x et en y
        int mouseX, mouseY; // Position de la souris
    public :
        // Constructeur
        Simulation();

        void run(); // Lance la simulation
        void initializeBoidsRandomly(int numBoids); // Initialiser les boids de manière aléatoire
        int findMinDivisor(int number, int minSize); // Trouver le diviseur min d'un nombre supérieur ou égal à un certain seuil

        // AFFICHAGE
        void updateDisplay() const; // Afficher la simulation

        // CONTRÔLE BOIDS
        inline void addBoid(float x_, float y_, float theta_); // Ajouter un boid à la simulation
        inline void removeBoid(int id); // Supprimer un boid de la simulation
        inline void reset(); // Réinitialiser la simulation

        // CONTRÔLE SIMULATION
        inline void handleKeyPress(int key); // Gérer les touches
        inline void togglePause(); // Gérer la pause de la simulation

        // FONCTIONS UTILES CUDA
        inline void allocateBoidDataOnGPU(); // Allouer la mémoire GPU
        inline void freeBoidDataOnGPU(); // Libérer la mémoire GPU
        inline void copyBoidDataToGPU(); // Transférer les données CPU -> GPU
        inline void copyBoidDataToCPU(); // Transférer les données GPU -> CPU
        inline void reallocate(); // Réallouer si la taille change

        ~Simulation();
};

#endif // SIMULATION_HPP
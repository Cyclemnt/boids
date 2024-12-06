#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <vector>

class Simulation {
    private : 
        // Poses
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> theta;
        // Interactions (couleur)
        std::vector<int> interaction;
        // Spatial hashing
        std::vector<int> cellCount; // Nombre de boids par cellule (initialement), devient la somme partielle
        std::vector<int> particleMap; // Indices des boids triés par cellule
        // Pointeurs GPU
        float* d_x;
        float* d_y;
        float* d_theta;
        int* d_interaction;
        int* d_cellCount;
        int* d_particleMap;
        unsigned char* d_image;
        // Divers
        bool paused; // état de la simulation (pause/play)
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
        void removeBoid(int id); // Supprimer un boid de la simulation
        void reset(); // Réinitialiser la simulation

        // CONTRÔLE SIMULATION
        void handleKeyPress(int key); // Gérer les touches
        void togglePause(); // Gérer la pause de la simulation

        // FONCTIONS UTILES CUDA
        void allocateBoidDataOnGPU(); // Allouer la mémoire GPU
        void freeBoidDataOnGPU(); // Libérer la mémoire GPU
        void copyBoidDataToGPU(); // Transférer les données CPU -> GPU
        void copyBoidDataToCPU(); // Transférer les données GPU -> CPU
        inline void reallocate(); // Réallouer si la taille change

        ~Simulation();
};

#endif // SIMULATION_HPP
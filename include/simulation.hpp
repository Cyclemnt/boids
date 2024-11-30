#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <opencv2/opencv.hpp>

class Simulation {
    private : 
        // Poses
        std::vector<float> x;
        std::vector<float> y;
        std::vector<float> theta;
        // Interactions
        std::vector<int> interaction;
        // Pointeurs GPU
        float* d_x;
        float* d_y;
        float* d_theta;
        int* d_interaction;
        
        bool paused; // état de la simulation (pause/play)
    public :
        // Constructeur
        Simulation();

        void run(); // Lance la simulation
        void initializeBoidsRandomly(int numBoids); // Méthode pour initialiser les boids de manière aléatoire

        // AFFICHAGE
        void updateDisplay() const; // Met à jour tous les boids et affiche la simulation
        inline void displayBoid(cv::Mat& image, int id) const; // Affiche chaque boid avec une couleur selon son interaction

        // CONTRÔLE BOIDS
        inline void addBoid(float x_, float y_, float theta_); // Méthode pour ajouter un boid à la simulation
        void removeBoid(int id); // Méthode pour supprimer un boid de la simulation
        void reset(); // Réinitialiser la simulation

        // CONTRÔLE SIMULATION
        void handleKeyPress(int key); // Méthode pour gérer les touches
        void togglePause(); // Méthode pour gérer la pause de la simulation

        // FONCTIONS UTILES CUDA
        void allocateBoidDataOnGPU(); // Fonction pour allouer la mémoire GPU
        void freeBoidDataOnGPU(); // Fonction pour libérer la mémoire GPU
        void copyBoidDataToGPU(); // Fonction pour transférer les données CPU -> GPU
        void copyBoidDataToCPU(); // Fonction pour transférer les données GPU -> CPU
        inline void reallocate(); // Fonction pour réallouer si la taille change

        ~Simulation();
};

#endif // SIMULATION_HPP
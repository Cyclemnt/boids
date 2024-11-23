#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "types.hpp"
#include <opencv2/opencv.hpp>

class Simulation {
    private : 
        Types::BoidData boids;
        const int envWidth, envHeight;
        const float timeStep;
        bool paused;
    public :
        // Constructeur
        Simulation(int envWidth_, int envHeight_, int timeStep_);

        // Lance la simulation
        void run();
        // Méthode pour initialiser les boids de manière aléatoire
        void initializeBoidsRandomly(int numBoids);
        // Méthode pour ajouter un boid à la simulation
        void addBoid(float x, float y, float theta);
        // Méthode pour supprimer un boid de la simulation
        void removeBoid(int id);
        // Réinitialiser la simulation
        void reset();

        // Méthode pour gérer les touches
        void handleKeyPress(int key);
        // Met à jour tous les boids et affiche la simulation
        void updateDisplay() const;
        // Affiche chaque boid avec une couleur selon son interaction
        void displayBoid(cv::Mat& image, int id) const;
        // Méthode pour gérer la pause de la simulation
        void togglePause();

        ~Simulation();
};

#endif // SIMULATION_HPP
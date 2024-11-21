#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include "zone.hpp"
#include <opencv2/opencv.hpp>

class Simulation {
    private : 
        int envWidth, envHeight, timeStep;
        std::vector<Boid*> boids;
        Zone* zoneptr;
        bool paused;
    public :
        // Constructeur
        Simulation(int envWidth_, int envHeight_, int timeStep_);

        // Lance la simulation
        void run();
        // Méthode pour ajouter un boid à la simulation
        void addBoid(vPose pose, double maxSpeed, double maxAngVelocity);
        // Méthode pour supprimer un boid de la simulation
        void removeNearestBoid(int x, int y);
        // Méthode pour initialiser les boids de manière aléatoire
        void initializeBoidsRandomly(int numBoids, double maxSpeed, double maxAngVelocity);
        // Méthode pour gérer les touches
        void handleKeyPress(int key);
        // Réinitialiser la simulation
        void reset();
        // Méthode pour gérer la pause de la simulation
        void togglePause();
        // Met à jour tous les boids et affiche la simulation
        void updateDisplay();
        // Affiche chaque boid avec une couleur selon son interaction
        void displayBoid(cv::Mat& image, const Boid* boid);
        // Méthode pour obtenir l'état de la simulation
        bool isPaused() const ;

        ~Simulation();
};

#endif // SIMULATION_HPP
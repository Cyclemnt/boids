#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include "zone.hpp"
#include <opencv2/opencv.hpp>

class Simulation {
    private : 
        int envWidth, envHeight, timeStep;
        std::vector<Boid*> boids;
        std::vector<Boid*> predators;
        Zone* zoneptr;
        Zone* zoneprdt;
        bool paused;
    public :
        // Constructeur
        Simulation(int envWidth_, int envHeight_, int timeStep_);

        // Lance la simulation
        void run();
        // Méthode pour ajouter un boid à la simulation
        void addBoid(vPose pose, double maxSpeed, double maxAngVelocity, int lifeTime);
        // Méthode pour supprimer un boid de la simulation
        void removeBoid();
        // Méthode pour supprimer un boid précis de la simulation
        void removeThisBoid(Boid* boid);
        // Méthode pour ajouter un predator à la simulation
        void addPredator(vPose pose, double maxSpeed, double maxAngVelocity, int lifeTime);
        // Méthode pour supprimer un predator de la simulation
        void removePredator();
        // Méthode pour supprimer un predator précis de la simulation
        void removeThisPredator(Boid* predator);
        // Méthode pour initialiser les boids de manière aléatoire
        void initializeBoidsRandomly(int numBoids, double maxSpeed, double maxAngVelocity, int lifeTime);
        // Méthode pour initialiser les predators de manière aléatoire
        void initializePredatorsRandomly(int numBoids, double maxSpeed, double maxAngVelocity, int lifeTime);
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
        // Affiche chaque predator avec une couleur selon son interaction
        void displayPredator(cv::Mat& image, const Boid* predator);
        // Méthode pour obtenir l'état de la simulation
        bool isPaused() const ;

        ~Simulation();
};

#endif // SIMULATION_HPP
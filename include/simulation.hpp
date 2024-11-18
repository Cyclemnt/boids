#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include "zone.hpp"
#include <vector>
#include <opencv2/opencv.hpp>

class Simulation {
    private : 
        int envWidth, envHeight, timeStep;
        std::vector<Boid*> boids;
        Zone* zoneptr;
    public :
        // Constructeur
        Simulation(int envWidth_, int envHeight_, int timeStep_);

        // Méthode pour ajouter un boid à la simulation
        void addBoid(vPose pose, int fov, double maxSpeed, double maxAngVelocity);
        // Méthode pour supprimer un boid de la simulation
        void removeBoid();
        // Méthode pour initialiser les boids de manière aléatoire
        void initializeBoidsRandomly(int numBoids, int fov, double maxSpeed, double maxAngVelocity);
        // Réinitialiser la simulation
        void reset();
        // Lance la simulation
        void run();
        // Met à jour tous les boids et affiche la simulation
        void update();
        // Affiche chaque boid avec une couleur selon son interaction
        void displayBoid(cv::Mat* image, const Boid* boid);
        // Méthode pour gérer la pause de la simulation
        void togglePause();
        // Méthode pour obtenir l'état de la simulation
        bool isPaused() const ;

        ~Simulation();
};

#endif // SIMULATION_HPP
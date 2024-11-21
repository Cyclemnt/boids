#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include "zone.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>



class Simulation {
    private : 
        int envWidth, envHeight, timeStep;
        std::vector<Boid*> boids;
        Boid* mouse;
        Zone* zoneptr;
        bool paused, mouseON;
    public :
        std::chrono::steady_clock::time_point lastMouseUpdateTime;  
        // Constructeur
        Simulation(int envWidth_, int envHeight_, int timeStep_);
        // Lance la simulation
        void run();
        //Mettre à jour la position de la souris 
        void updateMousePosition(int x, int y);
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
        // Méthode pour gérer la pause du suivi de la souris 
        void toggleMouse(); 
        // Met à jour tous les boids et affiche la simulation
        void updateDisplay();
        // Affiche chaque boid avec une couleur selon son interaction
        void displayBoid(cv::Mat& image, const Boid* boid);
        // Méthode pour obtenir l'état de la simulation
        bool isPaused() const ;

        ~Simulation();
};

#endif // SIMULATION_HPP
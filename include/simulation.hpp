#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include <vector>
#include <opencv2/opencv.hpp>

class Simulation {
    private : 
        double width;
        double height;
        std::vector<Boid*> boids;
    public :
        // Constructeur
        Simulation(double width_, double height_);

        // Méthode pour ajouter un boid à la simulation
        void addBoid(vPose pose, int fov, double maxSpeed, double maxAngVelocity);
        // Méthode pour supprimer un boid de la simulation
        void removeBoid();
        // Réinitialiser la simulation
        void reset();
        // Met à jour tous les boids et affiche la simulation
        void update();
        // Affiche chaque boid avec une couleur selon son interaction
        void displayBoid(cv::Mat* image, const Boid* boid);
        // Méthode pour gérer la pause de la simulation
        void togglePause();
        // Méthode pour obtenir l'état de la simulation
        bool isPaused() const ;
        // Méthode pour envoyer des informations aux boids
        void sendInfoToBoids();

        ~Simulation();
};

#endif // SIMULATION_HPP
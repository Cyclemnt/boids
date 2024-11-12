#include "../include/simulation.hpp"

Simulation::Simulation(double width_, double height_)
    : width(width_), height(height_) 
{
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
}

// Méthode pour ajouter un boid à la simulation
void Simulation::addBoid(vPose pose, int fov, double maxSpeed, double maxAngVelocity) {
    Boid* newBoid = new Boid(pose, fov, maxSpeed, maxAngVelocity);
    boids.push_back(newBoid);
}

// Méthode pour supprimer un boid de la simulation
void Simulation::removeBoid() {
    if (!boids.empty()) {
        delete boids.back();
        boids.pop_back();
    }
}

// Réinitialiser la simulation
void Simulation::reset() {
    for (Boid* boid : boids) {
        delete boid;
    }
    boids.clear();
}

// Met à jour tous les boids et affiche la simulation
void Simulation::update() {
    // Effacer l'image précédente
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    
    // Mettre à jour chaque boid
    for (Boid* boid : boids) {
        displayBoid(&image, boid); // Afficher le boid dans l'image
    }
    
    // Afficher l'image dans une fenêtre OpenCV
    cv::imshow("Simulation de Boids", image);
    cv::waitKey(10); // Pause pour rafraîchir l'affichage
}

// Affiche chaque boid avec une couleur selon son interaction
void Simulation::displayBoid(cv::Mat* image, const Boid* boid) {
    // Déterminer la couleur en fonction de l'interaction
    cv::Scalar color;
    switch (boid->getCurrentInteraction()) {
        case Interaction::DISTANCING:
            color = cv::Scalar(0, 0, 255); // Rouge
            break;
        case Interaction::ALIGNMENT:
            color = cv::Scalar(0, 255, 0); // Vert
            break;
        case Interaction::COHESION:
            color = cv::Scalar(255, 0, 0); // Bleu
            break;
    }

    // Dessiner le boid sous forme de point
    cv::Point2i position(static_cast<int>(boid->getPose().x), static_cast<int>(boid->getPose().y));
    cv::circle(*image, position, 3, color, -1); // Rayon de 3 pixels
}


// Méthode pour gérer la pause de la simulation
void Simulation::togglePause() {
}

bool Simulation::isPaused() const {
    return false;
}

// Méthode pour envoyer des informations aux boids
void Simulation::sendInfoToBoids() {
}

Simulation::~Simulation() {
    reset();
    cv::destroyAllWindows();
}

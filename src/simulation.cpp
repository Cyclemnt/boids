#include "../include/simulation.hpp"

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), zoneptr(nullptr) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    zoneptr = new Zone(1, 2, 3);
}

// Lance la Simulation
void Simulation::run() {
    addBoid({2, 2, 0}, M_PI, 0.5, 0.5);
    for (size_t i = 0; i < 1000; i++)
    {
        update();
        boids[0]->move(envWidth, envHeight);
    }
    
    /*
    for (int i = 0; i < boids.size(); i++) {
        for (auto interaction : {Interaction::DISTANCING, Interaction::ALIGNMENT, Interaction::COHESION}) {
            boids[i]->applyRules(interaction, zoneptr->getNearBoids(interaction, boids[i], boids));
        }
        boids[i]->move(envWidth, envHeight);
    }
    update();
    */
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
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    
    // Mettre à jour chaque boid
    for (Boid* boid : boids) {
        displayBoid(&image, boid); // Afficher le boid dans l'image
    }
    
    // Afficher l'image dans une fenêtre OpenCV
    cv::imshow("Simulation de Boids", image);
    cv::waitKey(timeStep); // Pause pour rafraîchir l'affichage
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
    cv::Point2i position(static_cast<int>(boid->getPose().x), envHeight - static_cast<int>(boid->getPose().y));
    cv::circle(*image, position, 3, color, -1); // Rayon de 3 pixels
}


// Méthode pour changer l'état de la simulation
void Simulation::togglePause() {
}
// Méthode pour obtenir l'état de la simulation
bool Simulation::isPaused() const {
    return false;
}

Simulation::~Simulation() {
    reset();
    cv::destroyAllWindows();
}

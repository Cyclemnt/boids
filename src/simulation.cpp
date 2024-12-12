#include "../include/simulation.hpp"
#include <random>

// Paramètres
#define NUM_BOIDS 800       // Nombre de Boids initialisés au début
#define SPEED 140           // Vitesse des Boids (px/s)
#define ANG_V (2 * M_PI)    // Vitesse angulaire maximum des Boids (rad/s)
#define FOV 5               // Angle de vue des Boids (rad)
// Rayons des règles d'interaction (px)
#define R_DISTANCING 10
#define R_ALIGNMENT 40
#define R_COHESINON 90
// Poids des règles d'interaction
#define WEIGHT_DISTANCING 0.05
#define WEIGHT_ALIGNMENT 0.05
#define WEIGHT_COHESION 0.0005

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), zoneptr(nullptr), paused(false) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    // Instanciation d'une zone avec rayons et fov en paramètres
    zoneptr = new Zone(R_DISTANCING, R_ALIGNMENT, R_COHESINON, FOV);
}

// Lance la Simulation
void Simulation::run() {
    // Initialiser des boids avec des positions aléatoires
    initializeBoidsRandomly(NUM_BOIDS, SPEED, ANG_V);

    // Boucle principale
    while (true) {
        // Gestion des entrées clavier
        int key = cv::waitKey(1); // Remplacer "timeStep" ici par 1 pour une simulation plus fluide, mais moins juste
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        // Si en pause, ne pas mettre à jour la simulation
        if (paused) continue;

        // Parcourir tous les boids
        for (int i = 0; i < boids.size(); i++) {
            std::vector<std::vector<Boid*>> neighbors = zoneptr->getNearBoids(boids[i], boids, envWidth, envHeight);
            boids[i]->applyRules(neighbors, WEIGHT_DISTANCING, WEIGHT_ALIGNMENT, WEIGHT_COHESION, envWidth, envHeight);
            boids[i]->move(envWidth, envHeight);
        }
        updateDisplay();
    }
}

// Méthode pour ajouter un boid à la simulation
void Simulation::addBoid(vPose pose, double maxSpeed, double maxAngVelocity) {
    Boid* newBoid = new Boid(pose, maxSpeed, maxAngVelocity);
    newBoid->setTimeStep(timeStep);
    boids.push_back(newBoid);
}

// Méthode pour supprimer un boid de la simulation
void Simulation::removeBoid() {
    if (!boids.empty()) {
        delete boids.back();
        boids.pop_back();
    }
}

// Méthode pour initialiser les boids de manière aléatoire
void Simulation::initializeBoidsRandomly(int numBoids, double maxSpeed, double maxAngVelocity) {
    // Création d'un moteur aléatoire avec une graine unique
    std::random_device rd;  // Génére une graine à partir de l'environnement
    std::mt19937 gen(rd()); // Mersenne Twister : générateur de nombres pseudo-aléatoires
    std::uniform_real_distribution<> xDist(0, envWidth);
    std::uniform_real_distribution<> yDist(0, envHeight);
    std::uniform_real_distribution<> thetaDist(0, 2 * M_PI);
    std::uniform_real_distribution<> offsetDist(-rand(), rand());
    double offsetTheta = Types::customMod(offsetDist(gen), 2*M_PI);
    
    for (int i = 0; i < numBoids; ++i) {
        vPose newPose;
        newPose.x = xDist(gen);  // Position x aléatoire
        newPose.y = yDist(gen);  // Position y aléatoire
        newPose.theta = thetaDist(gen) + offsetTheta;  // Orientation aléatoire
        addBoid(newPose, maxSpeed, maxAngVelocity);
    }
}

// Méthode pour gérer les touches
void Simulation::handleKeyPress(int key) {
    switch (key) {
        case 'p': // Pause ou reprise
            togglePause();
            std::cout << (paused ? "Simulation en pause." : "Simulation reprise.") << std::endl;
            break;
        case 'r': // Réinitialiser
            reset();
            std::cout << "Simulation réinitialisée." << std::endl;
            break;
        case '+': // Ajouter un boid
            initializeBoidsRandomly(1, SPEED, ANG_V);
            std::cout << "Boid ajouté." << std::endl;
            break;
        case '-': // Supprimer un boid
            removeBoid();
            std::cout << "Boid supprimé." << std::endl;
            break;
        case 27: // Échapper (ESC) pour quitter
            std::cout << "Simulation terminée." << std::endl;
            exit(0);
    }
}

// Réinitialiser la simulation
void Simulation::reset() {
    for (Boid* boid : boids) {
        delete boid;
    }
    boids.clear();
}

// Méthode pour basculer l'état de pause
void Simulation::togglePause() {
    paused = !paused;
}

// Met à jour tous les boids et affiche la simulation
void Simulation::updateDisplay() {
    // Effacer l'image précédente
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    
    // Mettre à jour chaque boid
    for (Boid* boid : boids) {
        displayBoid(image, boid); // Afficher le boid dans l'image
    }
    
    // Afficher l'image dans une fenêtre OpenCV
    cv::imshow("Simulation de Boids", image);
}

// Affiche chaque boid avec une couleur selon son interaction
void Simulation::displayBoid(cv::Mat& image, const Boid* boid) {
    // Déterminer la couleur en fonction de l'interaction
    cv::Scalar color;
    Interaction currentInteraction = boid->getCurrentInteraction();
    switch (currentInteraction) {
        case Interaction::DISTANCING: color = cv::Scalar(0, 0, 255);   break;
        case Interaction::ALIGNMENT:  color = cv::Scalar(0, 255, 0);   break;
        case Interaction::COHESION:   color = cv::Scalar(255, 0, 0);   break;
        case Interaction::NONE:       color = cv::Scalar(127, 127, 0); break;
    }

    // Dessiner le boid sous forme de triangle isocèle
    vPose pose = boid->getPose();
    double x = pose.x;
    double y = pose.y;
    double size = 5.0;         // Taille globale du triangle
    double angle = pose.theta; // Orientation du boid en radians

    // Calcul et dessin en une "pseudo-ligne"
    cv::fillPoly(
        image,
        {std::vector<cv::Point>{
            cv::Point(x + size * cos(angle), y + size * sin(angle)),                       // Sommet avant (pointe)
            cv::Point(x + size * cos(angle + CV_PI * 3 / 4), y + size * sin(angle + CV_PI * 3 / 4)), // Coin gauche
            cv::Point(x + size * cos(angle - CV_PI * 3 / 4), y + size * sin(angle - CV_PI * 3 / 4))  // Coin droit
        }},
        color
    );
}

// Vérifie si la simulation est en pause
bool Simulation::isPaused() const {
    return paused;
}

Simulation::~Simulation() {
    reset();
    cv::destroyAllWindows();
}

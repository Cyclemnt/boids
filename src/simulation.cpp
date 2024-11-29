#include "../include/simulation.hpp"
#include <random>

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), predators({}), zoneptr(nullptr), zoneprdt(nullptr), paused(false) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    zoneptr = new Zone(10, 30, 80, 0, 80, 2, 5, 2 * M_PI);
    zoneprdt = new Zone(0, 0, 80, 25, 10, 2, 2 * M_PI, 5);
    // légende des zones(distancing, alignement, cohesion, predation, fled, catch, fov, instinct)
}

// Lance la Simulation
void Simulation::run() {
    // Initialiser des boids avec des positions aléatoires
    initializeBoidsRandomly(NUM_BOIDS, SPEED, ANG_V);

    // Boucle principale
    while (true) {
        // Gestion des entrées clavier
        int key = cv::waitKey(timeStep); // Remplacer "timeStep" ici par 1 pour une simulation plus fluide, mais moins juste
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        // Si en pause, ne pas mettre à jour la simulation
        if (paused) continue;

        // Parcourir tous les boids
        for (int i = 0; i < boids.size(); i++) {
            std::vector<std::vector<Boid*>> neighbors = zoneptr->getNearBoids(boids[i], boids, envWidth, envHeight);
            boids[i]->applyRules(neighbors, WEIGHT_DISTANCING, WEIGHT_ALIGNMENT, WEIGHT_COHESION);
            boids[i]->move(envWidth, envHeight);
        }
        updateDisplay();
    }
}


// Méthode pour initialiser les boids de manière aléatoire
void Simulation::initializeBoidsRandomly(int numBoids, double maxSpeed, double maxAngVelocity, int lifeTime) {
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
        addBoid(newPose, maxSpeed, maxAngVelocity, lifeTime);
    }
}

// Méthode pour initialiser les predators de manière aléatoire
void Simulation::initializePredatorsRandomly(int numBoids, double maxSpeed, double maxAngVelocity, int lifeTime) {
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
        addPredator(newPose, maxSpeed, maxAngVelocity, lifeTime);
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
            initializeBoidsRandomly(1, 200, 2*M_PI, 500);
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
    for (Boid* predator : predators) {
    delete predator;
    }
    predators.clear();
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

    // Mettre à jour chaque predator
    for (Boid* predator : predators) {
        displayPredator(image, predator); // Afficher le boid dans l'image
    }
    
    // Afficher l'image dans une fenêtre OpenCV
    cv::namedWindow("Simulation", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Simulation", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow("Simulation", image);
}

// Affiche chaque boid avec une couleur selon son interaction
void Simulation::displayBoid(cv::Mat& image, const Boid* boid) {
    // Déterminer la couleur en fonction de l'interaction
    cv::Scalar color;
    Interaction currentInteraction = boid->getCurrentInteraction();
    switch (currentInteraction) {
        case Interaction::DISTANCING:
            color = cv::Scalar(0, 255, 255); // Jaune
            break;
        case Interaction::FLED:
            color = cv::Scalar(230, 216, 173); // Bleu clair
            break;
        case Interaction::ALIGNMENT:
            color = cv::Scalar(0, 255, 0); // Vert
            break;
        case Interaction::COHESION:
            color = cv::Scalar(227, 0, 0); // Bleu
            break;
        case Interaction::NONE:
            color =cv::Scalar(127,127,127); // Gris
            break;
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

void Simulation::displayPredator(cv::Mat& image, const Boid* predator) {
    // Déterminer la couleur en fonction de l'interaction
    cv::Scalar color;
    Interaction currentInteraction = predator->getCurrentInteraction();
    switch (currentInteraction) {
        case Interaction::FLED:
            color = cv::Scalar(0, 255, 255); // Jaune
            break;
        case Interaction::PREDATION:
            color = cv::Scalar(0, 0, 201); // Rouge
            break;
        case Interaction::COHESION:
            color = cv::Scalar(201, 0, 201); // Violet
            break;
        case Interaction::NONE:
            color = cv::Scalar(127, 127, 127); // Gris
            break;
    }

    // Dessiner le boid sous forme de triangle isocèle
    vPose pose = predator->getPose();
    double x = pose.x;
    double y = pose.y;
    double size = 10;         // Taille globale du triangle
    double angle = pose.theta; // Orientation du boid en radians

//  // Créer une variable statique pour stocker les positions historiques
//  static std::map<const Boid*, std::vector<cv::Point>> predatorTrails;

// // Paramètres
// const double trailMaxLength = 15;  // Nombre maximum de positions stockées
// const double maxTrailAge = 1.0;    // Durée de vie d'un point en secondes
// const double timeStepIncrement = 0.05; // Incrément d'âge par appel

// // Récupérer ou créer la traînée pour le boid actuel
// auto& trail = predatorTrails[predator];

// // Ajouter la position actuelle si elle est différente de la précédente
// if (trail.empty() || cv::norm(trail.back() - cv::Point(x, y)) > 2.0) {
//     trail.push_back(cv::Point(x, y)); // Nouveau point
// }

// // Supprimer les anciens points si nécessaire
// if (trail.size() > trailMaxLength) {
//     trail.erase(trail.begin());
// }

// // Dessiner un trait à chaque ancienne position
// for (size_t i = 0; i < trail.size(); ++i) {
//     // Calculer la couleur basée sur l'âge du point
//     double ageFactor = std::min(1.0, (double)i / trailMaxLength);
//     cv::Scalar fadedColor = cv::Scalar(
//         std::max(0.0, color[0] - 150 * ageFactor), // Rouge
//         std::max(0.0, color[1] - 150 * ageFactor), // Vert
//         std::max(0.0, color[2] - 150 * ageFactor), // Bleu
//         255                                      // Alpha (opacité)
//     );

//     // Dessiner un petit trait (ligne d'un pixel de long) à cet emplacement
//     cv::line(image, trail[i], trail[i], fadedColor, 1);
// }

    // Dessiner le boid sous forme de triangle isocèle
    cv::fillPoly(
    image,
    {std::vector<cv::Point>{
        cv::Point(x + size * cos(angle), y + size * sin(angle)),// Sommet avant (pointe)
        cv::Point(x + size * 0.7 * cos(angle + CV_PI * 3 / 4), y + size * 0.7 * sin(angle + CV_PI * 3 / 4)), // Coin arrière gauche
        cv::Point(x + size * 0.3 * cos(angle + CV_PI), y + size * 0.3 * sin(angle + CV_PI)), // Pointe arrière
        cv::Point(x + size * 0.7 * cos(angle - CV_PI * 3 / 4), y + size * 0.7 * sin(angle - CV_PI * 3 / 4))  // Coin arrière droit
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

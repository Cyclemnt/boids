#include "../include/simulation.hpp"
#include <random>

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), zoneptr(nullptr), zoneprdt(nullptr), paused(false) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    zoneptr = new Zone(10, 40, 90, 0, 80, 2 * M_PI, 2 * M_PI);
    zoneprdt = new Zone(0, 0, 60, 30, 40, 2 * M_PI, 2 * M_PI);
}

// Lance la Simulation
void Simulation::run() {
    // Initialiser 50 boids avec des positions et paramètres aléatoires
    initializeBoidsRandomly(500, 200, M_PI);
    initializePredatorsRandomly(1, 250, M_PI);
    double speedVar = 1.5;
    double velocityVar = 1.25;
    double originalSpeed = boids[0]->getSpeed();
    double originalAngVelocity = boids[0]->getAngVelocity();
    // Lancer la simulation
    while (true) {
        // Gestion des entrées clavier
        int key = cv::waitKey(timeStep);
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        // Si en pause, ne pas mettre à jour la simulation
        if (paused) continue;

        for (int i = 0; i < boids.size(); i++) {
            bool hasInteraction = false;
            for (auto interaction : {Interaction::FLED, Interaction::DISTANCING, Interaction::ALIGNMENT, Interaction::COHESION}) {
                auto neighbors = zoneptr->getNearBoids(interaction, boids[i], boids, predators[i], predators, envWidth, envHeight);
                if (!neighbors.empty()) {
                    if (interaction == Interaction::FLED) {
                        boids[i]->setSpeed(originalSpeed * speedVar);
                        boids[i]->setAngVelocity(originalAngVelocity * velocityVar);

                        boids[i]->applyRules(interaction, neighbors);
                    }
                    else {
                        boids[i]->applyRules(interaction, neighbors);
                        break;
                    }
                    hasInteraction = true;
                    break; // Si une interaction est trouvée, arrêter de vérifier les autres
                }
            }

            // Si aucune interaction, appliquer NONE
            if (!hasInteraction) {
                boids[i]->applyRules(Interaction::NONE, {});
            }

            // Mettre à jour la position
                boids[i]->move(envWidth, envHeight);
                boids[i]->setSpeed(originalSpeed);  // Réinitialiser la vitesse
                boids[i]->setAngVelocity(originalAngVelocity);  // Réinitialiser la vitesse angulaire
        }
        for (int i = 0; i < predators.size(); i++) {
            for (auto interaction : {Interaction::FLED, Interaction::NONE}) {
                auto neighbors = zoneprdt->getNearBoids(interaction, predators[i], predators, predators[i], predators, envWidth, envHeight); 
                if (!neighbors.empty()) {
                    predators[i]->applyRules(interaction, neighbors);
                    break;
                }
            }
            for (auto interaction : {Interaction::PREDATION, Interaction::COHESION, Interaction::NONE}) {
                auto neighbors = zoneprdt->getNearBoids(interaction, boids[i], boids, predators[i], predators, envWidth, envHeight); 
                if (!neighbors.empty()) {
                    predators[i]->applyRules(interaction, neighbors);
                    break;
                }
            }
            predators[i]->move(envWidth, envHeight);
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

// Méthode pour ajouter un predator à la simulation
void Simulation::addPredator(vPose pose, double maxSpeed, double maxAngVelocity) {
    Boid* newPredator = new Boid(pose, maxSpeed, maxAngVelocity);
    newPredator->setTimeStep(timeStep);
    predators.push_back(newPredator);
}

// Méthode pour supprimer un predator de la simulation
void Simulation::removePredator() {
    if (!predators.empty()) {
        delete predators.back();
        predators.pop_back();
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

// Méthode pour initialiser les boids de manière aléatoire
void Simulation::initializePredatorsRandomly(int numBoids, double maxSpeed, double maxAngVelocity) {
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
        addPredator(newPose, maxSpeed, maxAngVelocity);
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
            initializeBoidsRandomly(1, 200, 2*M_PI);
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
            color = cv::Scalar(0, 0, 255); // Rouge
            break;
        case Interaction::FLED:
            color = cv::Scalar(230, 216, 173); // Bleu clair
            break;
        case Interaction::ALIGNMENT:
            color = cv::Scalar(0, 255, 0); // Vert
            break;
        case Interaction::COHESION:
            color = cv::Scalar(255, 0, 0); // Bleu
            break;
        case Interaction::NONE:
            color =cv::Scalar(127,127,0); // Bleu-Vert 
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
            color = cv::Scalar(0, 0, 255); // Rouge
            break;
        case Interaction::PREDATION:
            color = cv::Scalar(0, 0, 130); // rouge foncé
            break;
        case Interaction::COHESION:
            color = cv::Scalar(255, 0, 0); // Bleu
            break;
        case Interaction::NONE:
            color =cv::Scalar(127,127,0); // Bleu-Vert 
            break;
    }

    // Dessiner le boid sous forme de triangle isocèle
    vPose pose = predator->getPose();
    double x = pose.x;
    double y = pose.y;
    double size = 30;         // Taille globale du triangle
    double angle = pose.theta; // Orientation du boid en radians

    // Calcul et dessin en une "pseudo-ligne"
    cv::fillPoly(
    image,
        {std::vector<cv::Point>{
            cv::Point(x + size * cos(angle), y + size * sin(angle)),                       // Sommet avant (pointe)
            cv::Point(x + size * 0.5 * cos(angle + CV_PI * 3 / 4), y + size * 0.5 * sin(angle + CV_PI * 3 / 4)), // Coin gauche
            cv::Point(x + size * 0.5 * cos(angle - CV_PI * 3 / 4), y + size * 0.5 * sin(angle - CV_PI * 3 / 4))  // Coin droit
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

#include "../include/simulation.hpp"
#include <random>

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), predators({}), zoneptr(nullptr), zoneprdt(nullptr), paused(false) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    zoneptr = new Zone(10, 30, 80, 0, 80, 1, 5, 2 * M_PI);
    zoneprdt = new Zone(0, 0, 80, 30, 10, 0, 2 * M_PI, 5);
    // légende des zones(distancing, alignement, cohesion, predation, fled, catch, fov, instinct)
}

// Lance la Simulation
void Simulation::run() {
    // Initialiser 50 boids avec des positions et paramètres aléatoires
    initializeBoidsRandomly(300, 200, 2 * M_PI);
    initializePredatorsRandomly(1, 300, 2 * M_PI);
    double speedVar = 2;
    double velocityVar = 2;
    double originalSpeed = boids[0]->getSpeed();
    double originalAngVelocity = boids[0]->getAngVelocity();
    // Lancer la simulation
    while (true) {
        // Gestion des entrées clavier
        int key = cv::waitKey(timeStep);
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        // Si en pause, ne pas mettre à jour la simulation
        if (paused) continue;
        // simulation pour les boids
        for (int i = 0; i < boids.size(); i++) {
            bool hasInteraction = false;
            for (auto interaction : {Interaction::CATCH, Interaction::FLED, Interaction::DISTANCING, Interaction::ALIGNMENT, Interaction::COHESION}) {
                auto neighbors = (interaction == Interaction::FLED || interaction == Interaction::CATCH) ? zoneptr->getNearBoids(interaction, boids[i], boids, predators, predators, envWidth, envHeight) 
                                                                                                        :zoneptr->getNearBoids(interaction, boids[i], boids, boids, predators, envWidth, envHeight);   
                if (!neighbors.empty()) {
                    if (interaction == Interaction::CATCH) {
                        vPose boidPose = boids[i]->getPose();
                        removeThisBoid(boids[i]);
                        addPredator(boidPose, 300, 2 * M_PI);
                        break;
                    } else {
                        boids[i]->applyRules(interaction, neighbors);
                        hasInteraction = true;
                        if (interaction == Interaction::FLED) {
                            boids[i]->setSpeed(originalSpeed * speedVar);
                            boids[i]->setAngVelocity(originalAngVelocity * velocityVar);
                        }
                        break; // Si une interaction est trouvée, arrêter de vérifier les autres
                    }
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
        // simulation pour les predators
        for (int i = 0; i < predators.size(); i++) {
            bool hasInteraction = false;
            for (auto interaction : {Interaction::FLED, Interaction::PREDATION, Interaction::COHESION}) {
                auto neighbors = (interaction == Interaction::FLED) ?  zoneprdt->getNearBoids(interaction, predators[i], predators, predators, predators, envWidth, envHeight) 
                                                                    :  zoneprdt->getNearBoids(interaction, predators[i], predators, boids, predators, envWidth, envHeight);
                if (!neighbors.empty()) {
                    predators[i]->applyRules(interaction, neighbors);
                    hasInteraction = true;
                    break; // Si une interaction est trouvée, arrêter de vérifier les autres
                }
            }

            // Si aucune interaction, appliquer NONE
            if (!hasInteraction) {
                boids[i]->applyRules(Interaction::NONE, {});
            }

            // Mettre à jour la position
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

// Méthode pour supprimer un boid précis de la simulation
void Simulation::removeThisBoid(Boid* boid) {
    auto it = std::find(boids.begin(), boids.end(), boid);
    if (it != boids.end()) {
        boids.erase(it);
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

// Méthode pour supprimer un predator précis de la simulation
void Simulation::removeThisPredator(Boid* predator) {
    auto it = std::find(predators.begin(), predators.end(), predator);
    if (it != predators.end()) {
        predators.erase(it);
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

    // Créer une variable locale pour la traînée et son âge
    static std::map<const Boid*, std::vector<cv::Point>> predatorTrails;
    static std::map<const Boid*, std::vector<double>> predatorTrailTimes;

    // Créer ou récupérer la traînée du prédator actuel
    auto& trail = predatorTrails[predator];
    auto& trailTimes = predatorTrailTimes[predator];

    const double trailMaxLength = 15;  // Nombre maximum de points de traînée
    const double maxTrailAge = 1.0;   // Temps après lequel les points disparaissent

    // Ajouter la position actuelle à la traînée
    trail.push_back(cv::Point(x, y));
    trailTimes.push_back(0.0);  // L'âge du segment commence à 0

    // Si la taille de la traînée dépasse la limite, supprimer les anciens points
    if (trail.size() > trailMaxLength) {
        trail.erase(trail.begin());
        trailTimes.erase(trailTimes.begin());
    }

    // Mettre à jour les temps des points de traînée
    for (size_t i = 0; i < trailTimes.size(); ++i) {
        trailTimes[i] += 0.05;  //augmenter l'âge de 0.05 par appel
    }

    // Dessiner des points de traînée avec des couleurs qui deviennent plus sombres
    for (size_t i = 0; i < trail.size(); ++i) {
        double ageFactor = std::min(1.0, trailTimes[i] / maxTrailAge);
        
        // Plus le point est vieux, plus il devient sombre
        cv::Scalar fadedColor = cv::Scalar(
            std::max(0.0, color[0] - 150 * ageFactor), // rouge
            std::max(0.0, color[1] - 150 * ageFactor), // vert
            std::max(0.0, color[2] - 150 * ageFactor), // bleu
            255 // Maintenir l'opacité complète
        );

        // Dessiner un cercle à chaque position de la traînée
        cv::circle(image, trail[i], 1, fadedColor, -1);  // Rayon fixe à 1 pixel
    }

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

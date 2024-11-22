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
    // Initialiser 50 boids avec des positions et paramètres aléatoires
    double speedVar = 2;
    double velocityVar = 2;
    double originalSpeed = 75;
    double originalAngVelocity =  2 * M_PI;
    int lifePred = 400;
    int lifeBoid = 100; // fatigue boid 
    initializeBoidsRandomly(400, originalSpeed, originalAngVelocity, lifeBoid);
    initializePredatorsRandomly(1, 170, 2 * M_PI, lifePred);
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
                        addPredator(boidPose, 150, 2 * M_PI, lifePred);
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
            for (auto interaction : {Interaction::CATCH, Interaction::FLED, Interaction::PREDATION, Interaction::COHESION}) {
                auto neighbors = (interaction == Interaction::FLED) ?  zoneprdt->getNearBoids(interaction, predators[i], predators, predators, predators, envWidth, envHeight) 
                                                                    :  zoneprdt->getNearBoids(interaction, predators[i], predators, boids, predators, envWidth, envHeight);
                if (!neighbors.empty()) {
                    if (interaction == Interaction::CATCH) {
                        predators[i]->setLifeTime(lifePred);
                        hasInteraction = true;
                        break;
                    } else {
                        predators[i]->applyRules(interaction, neighbors);
                        hasInteraction = true;
                        break; // Si une interaction est trouvée, arrêter de vérifier les 
                    }
                }
            }

            // Si aucune interaction, appliquer NONE
            if (!hasInteraction) {
                boids[i]->applyRules(Interaction::NONE, {});
            }

            // Mettre à jour la position
            predators[i]->move(envWidth, envHeight);

            // Décrémenter le temps de vie du predator
            predators[i]->setLifeTime(predators[i]->getLifeTime() - 1);

            // Supprimez le predator si son survivalTime atteint zéro
            if (predators[i]->getLifeTime() <= 0) {
                vPose predatorPose = predators[i]->getPose();
                removeThisPredator(predators[i]);
                addBoid(predatorPose, originalSpeed, originalAngVelocity, lifeBoid);
                i--;    // Ajustez l'indice après suppression
                }
        }
        updateDisplay();
    }
}

// Méthode pour ajouter un boid à la simulation
void Simulation::addBoid(vPose pose, double maxSpeed, double maxAngVelocity, int lifeTime) {
    Boid* newBoid = new Boid(pose, maxSpeed, maxAngVelocity, lifeTime);
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
void Simulation::addPredator(vPose pose, double maxSpeed, double maxAngVelocity, int lifeTime) {
    Boid* newPredator = new Boid(pose, maxSpeed, maxAngVelocity, lifeTime);
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

// Méthode pour initialiser les boids de manière aléatoire
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

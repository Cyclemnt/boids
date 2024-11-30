#include "../include/simulation.hpp"
#include <random>

// Paramètres
    //boids
    #define NUM_BOIDS 800       // Nombre de Boids initialisés au début
    #define SPEED_B 200           // Vitesse des Boids (px/s)
    #define ANG_V_B (2 * M_PI)    // Vitesse angulaire maximum des Boids (rad/s)
    #define FOV_B 5               // Angle de vue des Boids (rad)
    #define INSTINCT_B 6          // Angle de déctection des prédateurs
    #define LIFE_B 0              // temps de vie d'un boid ( inutilisé )
    //predators
    #define NUM_PREDATORS 5      // Nombre de predators initialisés au début
    #define SPEED_P 300           // Vitesse des predators (px/s)
    #define ANG_V_P (2 * M_PI)    // Vitesse angulaire maximum des predators (rad/s)
    #define FOV_P 5               // Angle de vue des predators (rad)
    #define INSTINCT_P 5          // Angle de déctection des predators
    #define LIFE_P 200            // temps de vie d'un predator
// Rayons des règles d'interaction (px)
    //boids 
    #define R_DISTANCING_B 10
    #define R_ALIGNMENT_B 40
    #define R_COHESINON_B 90
    #define R_FLED_B 50
    #define R_PREDATION_B 0
    #define R_CATCH_B 1
    #define R_FEED_B 50
    #define R_BREED_B 0
    //predators
    #define R_DISTANCING_P 0
    #define R_ALIGNMENT_P 0
    #define R_COHESINON_P 90
    #define R_FLED_P 10
    #define R_PREDATION_P 40
    #define R_CATCH_P 0
    #define R_FEED_P 0
    #define R_BREED_P 0
    //food
    #define R_BREED_F 1
// Poids des règles d'interaction
    //boids
    #define WEIGHT_DISTANCING_B 0.05
    #define WEIGHT_ALIGNMENT_B 0.05
    #define WEIGHT_COHESION_B 0.0005
    #define WEIGHT_FLED_B 0.3
    #define WEIGHT_PREDATION_B 0
    #define WEIGHT_CATCH_B 0
    #define WEIGHT_FEED_B 0.2
    #define WEIGHT_BREED_B 0
    #define BOOST_SPEED 2
    #define BOOST_ANGV 2
    //predators
    #define WEIGHT_DISTANCING_P 0
    #define WEIGHT_ALIGNMENT_P 0
    #define WEIGHT_COHESION_P 0.005
    #define WEIGHT_FLED_P 0.05
    #define WEIGHT_PREDATION_P 0.07
    #define WEIGHT_CATCH_P 0
    #define WEIGHT_FEED_P 0
    #define WEIGHT_BREED_P 0

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), predators({}), zoneptr(nullptr), zoneprdt(nullptr), paused(false) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    zoneptr = new Zone(R_DISTANCING_B, R_ALIGNMENT_B, R_COHESINON_B, R_PREDATION_B, R_FLED_B, R_CATCH_B, R_FEED_B, R_BREED_B, FOV_B, INSTINCT_B);
    zoneprdt = new Zone(R_DISTANCING_P, R_ALIGNMENT_P, R_COHESINON_P, R_PREDATION_P, R_FLED_P, R_CATCH_P, R_FEED_P, R_BREED_P, FOV_P, INSTINCT_P);
    zonef = new Zone(0,0,0,0,0,0,0,R_BREED_F,0,0);
}

// Lance la Simulation
void Simulation::run() {
    // Initialiser des boids avec des positions aléatoires
    initializeBoidsRandomly(NUM_BOIDS, SPEED_B, ANG_V_B, LIFE_B);
    initializePredatorsRandomly(NUM_PREDATORS, SPEED_P, ANG_V_P, LIFE_P);
    // Boucle principale
    while (true) {
        // Gestion des entrées clavier
        int key = cv::waitKey(1); // Remplacer "timeStep" ici par 1 pour une simulation plus fluide, mais moins juste
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        // Si en pause, ne pas mettre à jour la simulation
        if (paused) continue;

        // Parcourir tous les boids
        for (int i = 0; i < boids.size(); i++) {
            std::vector<std::vector<Boid*>> neighbors = zoneptr->getNearBoids(boids[i], boids, predators, foods, envWidth, envHeight);
            boids[i]->applyRules(neighbors, WEIGHT_DISTANCING_B, WEIGHT_ALIGNMENT_B, WEIGHT_COHESION_B, WEIGHT_FLED_B, WEIGHT_PREDATION_B, WEIGHT_CATCH_B, WEIGHT_FEED_B, WEIGHT_BREED_B, envWidth, envHeight);
        if (boids[i]->getCurrentInteraction() == Interaction::CATCH) {        // disparition si attrapé
            vPose boidPose = boids[i]->getPose();
            removeThisBoid(boids[i]);
            addPredator(boidPose, SPEED_P, ANG_V_P, LIFE_P);
            break;
        } else {
            if (boids[i]->getCurrentInteraction() == Interaction::FLED) {     // fuite plus rapide
                boids[i]->setSpeed(SPEED_B * BOOST_SPEED);
                boids[i]->setAngVelocity(ANG_V_B * BOOST_ANGV);
            }
            boids[i]->move(envWidth, envHeight);
            boids[i]->setSpeed(SPEED_B);
            boids[i]->setAngVelocity(ANG_V_B);
            }
        }
        // Parcourir tous les predators
        for (int i = 0; i < predators.size(); i++) {
            std::vector<std::vector<Boid*>> neighbors = zoneprdt->getNearBoids(predators[i], boids, predators, foods, envWidth, envHeight);
            predators[i]->applyRules(neighbors, WEIGHT_DISTANCING_P, WEIGHT_ALIGNMENT_P, WEIGHT_COHESION_P, WEIGHT_FLED_P, WEIGHT_PREDATION_P, WEIGHT_CATCH_P, WEIGHT_FEED_P, WEIGHT_BREED_P, envWidth, envHeight);
            if (predators[i]->getLifeTime() <= 0) {     // supprimer le predator si sa vie est fini
                vPose predatorPose = predators[i]->getPose();
                removeThisPredator(predators[i]);
                addFood(predatorPose);
                break;
            } else {
            predators[i]->move(envWidth, envHeight);
            predators[i]->setLifeTime(predators[i]->getLifeTime() - 1);
            }
        }
        // Parcourir tous les foods
         for (int i = 0; i < foods.size(); i++) {
            std::vector<std::vector<Boid*>> neighbors = zonef->getNearBoids(foods[i], boids, predators, foods, envWidth, envHeight);
            foods[i]->applyRules(neighbors,0,0,0,0,0,0,0,0, envWidth, envHeight);
            if (foods[i]->getCurrentInteraction() == Interaction::BREED) {
                vPose foodPose = foods[i]->getPose();
                removeThisFood(foods[i]);
                addBoid(foodPose, SPEED_B, ANG_V_B, LIFE_B);
                break;
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

// Méthode pour ajouter un food à la simulation
void Simulation::addFood(vPose pose) {
    Boid* newFood = new Boid(pose, 0, 0, 0);
    newFood->setTimeStep(timeStep);
    foods.push_back(newFood);
}

// Méthode pour supprimer un food précis de la simulation
void Simulation::removeThisFood(Boid* food) {
    auto it = std::find(foods.begin(), foods.end(), food);
    if (it != foods.end()) {
        foods.erase(it);
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
    for (Boid* food : foods) {
    delete food;
    }
    foods.clear();
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

    // Mettre à jour chaque food
    for (Boid* food : foods) {
        displayFood(image, food); // Afficher le boid dans l'image
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
        case Interaction::FEED:
            color = cv::Scalar(128, 128, 0); // Bleu
            break;
        case Interaction::NONE:
            color =cv::Scalar(127,127,127); // Gris
            break;
    }

    // Dessiner une ligne
    vPose pose = boid->getPose();
    double x = pose.x;
    double y = pose.y;
    double size = 8;             // Taille 
    double angle = pose.theta;   // Orientation en radians

    cv::line(
        image,
        cv::Point(x, y), // Origine
        cv::Point(x + size * cos(angle), y + size * sin(angle)), // Direction
        color,
        1 // Épaisseur
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
    // Dessiner une ligne
    vPose pose = predator->getPose();
    double x = pose.x;
    double y = pose.y;
    double size = 10;            // Taille 
    double angle = pose.theta;   // Orientation en radians

    cv::line(
        image,
        cv::Point(x, y), // Origine
        cv::Point(x + size * cos(angle), y + size * sin(angle)), // Direction
        color,
        2 // Épaisseur
    );

}

// Affiche un élément de nourriture sous forme de petit cercle blanc
void Simulation::displayFood(cv::Mat& image, const Boid* food) {
    // Déterminer la position de la nourriture
    vPose pose = food->getPose();
    double x = pose.x;
    double y = pose.y;
    double radius = 2.0; // Rayon du cercle représentant la nourriture

    // Couleur blanche pour représenter la nourriture
    cv::Scalar color(255, 255, 255); // Blanc (BGR)

    // Dessiner un cercle rempli
    cv::circle(image, cv::Point(x, y), static_cast<int>(radius), color, cv::FILLED);
}

// Vérifie si la simulation est en pause
bool Simulation::isPaused() const {
    return paused;
}

Simulation::~Simulation() {
    reset();
    cv::destroyAllWindows();
}

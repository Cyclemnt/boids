#include "../include/simulation.hpp"

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), zoneptr(nullptr), paused(false) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    zoneptr = new Zone(10, 40, 90);
}

// Lance la Simulation
void Simulation::run() {
    // Initialiser 50 boids avec des positions et paramètres aléatoires
    initializeBoidsRandomly(1000, M_PI, 200, 2*M_PI);

    // Lancer la simulation
    while (true) {
        // Gestion des entrées clavier
        int key = cv::waitKey(timeStep);
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        // Si en pause, ne pas mettre à jour la simulation
        if (paused) continue;
        for (int i = 0; i < boids.size(); i++) {
            for (auto interaction : {Interaction::DISTANCING, Interaction::ALIGNMENT, Interaction::COHESION, Interaction::NONE}) {
                auto neighbors = zoneptr->getNearBoids(interaction, boids[i], boids, envWidth, envHeight); 
                if (!neighbors.empty()) {
                    boids[i]->applyRules(interaction, neighbors);
                    break;
                }
            }
            boids[i]->move(envWidth, envHeight);
        }
        update();
    }
}

// Méthode pour ajouter un boid à la simulation
void Simulation::addBoid(vPose pose, int fov, double maxSpeed, double maxAngVelocity) {
    Boid* newBoid = new Boid(pose, fov, maxSpeed, maxAngVelocity);
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
void Simulation::initializeBoidsRandomly(int numBoids, int fov, double maxSpeed, double maxAngVelocity) {
    srand(time(0));
    // Crée les boids
    bool isPositionValid = true;
    for (int i = 0; i < numBoids; ++i) {
        vPose newPose = {0, 0, 0};
        if (!boids.empty()) {isPositionValid = false;}

        while (!isPositionValid) {
            // Génère une nouvelle position aléatoire
            newPose.x = rand() % envWidth;
            newPose.y = rand() % envHeight;
            newPose.theta = rand() % 2*M_PI;

            // Vérifie que la position ne chevauche pas un boid existant
            if (!boids.empty()) {
                for (Boid* boid : boids) {
                    double dx = newPose.x - boid->getPose().x;
                    double dy = newPose.y - boid->getPose().y;
                    double distance = sqrt(dx * dx + dy * dy);

                    // Si la distance est assez grande, la position est valide
                    if (distance > 10) { // 10 est la distance minimale entre deux boids
                        isPositionValid = true;
                        break;
                    }
                }
            }
        }
        // Ajoute le boid avec la position valide
        addBoid(newPose, fov, maxSpeed, maxAngVelocity);
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
            initializeBoidsRandomly(1, M_PI, 100, 2);
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
void Simulation::update() {
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
        case Interaction::DISTANCING:
            color = cv::Scalar(0, 0, 255); // Rouge
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

// Vérifie si la simulation est en pause
bool Simulation::isPaused() const {
    return paused;
}

Simulation::~Simulation() {
    reset();
    cv::destroyAllWindows();
}

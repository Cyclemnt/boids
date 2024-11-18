#include "../include/simulation.hpp"
#include <random>

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), zoneptr(nullptr) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    zoneptr = new Zone(10, 20, 40);
}

// Lance la Simulation
void Simulation::run() {
    // Initialiser 50 boids avec des positions et paramètres aléatoires
    initializeBoidsRandomly(10, M_PI, 10, 1);

    // Lancer la simulation
    for (size_t i = 0; i < 1000; i++) {
        for (int i = 0; i < boids.size(); i++) {
            for (auto interaction : {Interaction::DISTANCING, Interaction::ALIGNMENT, Interaction::COHESION, Interaction::NOTHING}) {
                auto neighbors =zoneptr->getNearBoids(interaction, boids[i], boids, envWidth, envHeight, i); 
                boids[i]->applyRules(interaction, neighbors);
                if (neighbors.size() != 0) {
                    break;
                }
            }
            boids[i]->move(envWidth, envHeight);
        }update();
    }            

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

// Méthode pour initialiser les boids de manière aléatoire
void Simulation::initializeBoidsRandomly(int numBoids, int fov, double maxSpeed, double maxAngVelocity) {
    // Initialise un générateur de nombres aléatoires
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disX(0, envWidth);  // Pour la position en X
    std::uniform_real_distribution<> disY(0, envHeight); // Pour la position en Y
    std::uniform_real_distribution<> disTheta(-M_PI, M_PI); // Pour l'angle de direction

    // Crée les boids
    for (int i = 0; i < numBoids; ++i) {
        vPose newPose;
        bool isPositionValid = false;

        while (!isPositionValid) {
            // Génère une nouvelle position aléatoire
            newPose.x = disX(gen);
            newPose.y = disY(gen);
            newPose.theta = disTheta(gen);

            // Vérifie que la position ne chevauche pas un boid existant
            isPositionValid = true;
            for (Boid* boid : boids) {
                double dx = newPose.x - boid->getPose().x;
                double dy = newPose.y - boid->getPose().y;
                double distance = sqrt(dx * dx + dy * dy);

                // Si la distance est trop petite, la position est invalide
                if (distance < 10) {  // 10 est la distance minimale entre deux boids
                    isPositionValid = false;
                    break;
                }
            }
        }

        // Ajoute le boid avec la position valide
        addBoid(newPose, fov, maxSpeed, maxAngVelocity);
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
        displayBoid(image, boid); // Afficher le boid dans l'image
    }
    
    // Afficher l'image dans une fenêtre OpenCV
    cv::imshow("Simulation de Boids", image);
    cv::waitKey(timeStep); // Pause pour rafraîchir l'affichage
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
        case Interaction::NOTHING:
            color =cv::Scalar(127,127,0); // JSP quelle couleur 
            break;
    }

          // Dessiner le boid sous forme de triangle isocèle
    vPose pose = boid->getPose();
    double x = pose.x;
    double y = pose.y;
    double size = 5.0;           // Taille globale du triangle
    double angle = pose.theta; // Orientation du boid en radians

    // Calcul et dessin en une "pseudo-ligne"
    cv::fillPoly(
        image,
        {std::vector<cv::Point>{
            cv::Point(x + size * cos(angle), y + size * sin(angle)),                        // Sommet avant (pointe)
            cv::Point(x + size * cos(angle + CV_PI * 3 / 4), y + size * sin(angle + CV_PI * 3 / 4)), // Coin gauche
            cv::Point(x + size * cos(angle - CV_PI * 3 / 4), y + size * sin(angle - CV_PI * 3 / 4))  // Coin droit
        }},
        color
    );
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

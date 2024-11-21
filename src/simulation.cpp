#include "../include/simulation.hpp"
#include <random>

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), boids({}), zoneptr(nullptr), paused(false) {
    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    zoneptr = new Zone(10, 40, 90, 5);
}


void onMouse(int event, int x, int y, int flags, void* userdata) {
    Simulation* simulation = reinterpret_cast<Simulation*>(userdata);
    std::random_device rd;  // Génére une graine à partir de l'environnement
    std::mt19937 gen(rd()); // Mersenne Twister : générateur de nombres pseudo-aléatoires
    std::uniform_real_distribution<> thetaDist(0, 2 * M_PI);
    std::uniform_real_distribution<> offsetDist(-rand(), rand());
    double offsetTheta = Types::customMod(offsetDist(gen), 2*M_PI);
    switch (event) {
        case cv::EVENT_LBUTTONDOWN: // Ajouter un boid
            vPose pose;
            pose.x=x;
            pose.y=y;
            pose.theta = thetaDist(gen) + offsetTheta;  // Orientation aléatoire
            simulation->addBoid(pose, 200, M_PI);
            std::cout << "Boid ajouté " << std::endl;
            break;
        case cv::EVENT_RBUTTONDOWN: // Supprimer un boid
            simulation->removeNearestBoid(x,y);
            std::cout << "Boid supprimé." << std::endl;
            break;
    }
}

// Lance la Simulation
void Simulation::run() {
    // Initialiser des boids avec des positions aléatoires
    initializeBoidsRandomly(500, 200, 2*M_PI);

    // Lancer la simulation
    while (true) {
        // Gestion des entrées clavier
        int key = cv::waitKey(timeStep);
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        // Si en pause, ne pas mettre à jour la simulation
        if (paused) continue;
        cv::namedWindow("Simulation de Boids", cv::WINDOW_NORMAL);
        cv::setWindowProperty("Simulation de Boids", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::setMouseCallback("Simulation de Boids", onMouse, this);
        
        for (int i = 0; i < boids.size(); i++) {
            bool hasInteraction = false;
            for (auto interaction : {Interaction::DISTANCING, Interaction::ALIGNMENT, Interaction::COHESION}) {
                auto neighbors = zoneptr->getNearBoids(interaction, boids[i], boids, envWidth, envHeight);
                if (!neighbors.empty()) {
                    boids[i]->applyRules(interaction, neighbors);
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
void Simulation::removeNearestBoid(int x, int y) {
    if (!boids.empty()) {
        std::vector<std::pair<double,int>> distances;
        // Parcourir chaque boid pour calculer la distance torique
        for (int i = 0; i < boids.size(); i++) {
                // Calculer la distance en x en tenant compte de l'environnement torique
                double dx = std::fabs(x - boids[i]->getPose().x);
                // Calculer la distance en y en tenant compte de l'environnement torique
                double dy = std::fabs(y - boids[i]->getPose().y);
                // Calculer la distance euclidienne avec les distances minimales en x et y
                double distance = sqrt((dx * dx) + (dy * dy));
                
                distances.push_back(std::make_pair (distance,i));
        }
    std::sort(distances.begin(),distances.end()); 
    
    boids.erase(boids.begin()+distances[0].second);
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
    cv::namedWindow("Simulation de Boids", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Simulation de Boids", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
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

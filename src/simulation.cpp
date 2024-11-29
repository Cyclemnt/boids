#include "../include/simulation.hpp"
#include "../include/gpu_utils.cuh"
#include "../include/types.hpp"
#include <omp.h>
#include <random>
#include <iostream>
#include <chrono>

// Paramètres
#define NUM_BOIDS 2 * 32768      // Nombre de Boids initialisés au début
#define SPEED 35             // Vitesse des Boids (px/s)
#define ANG_V (2 * M_PIf)    // Vitesse angulaire maximum des Boids (rad/s)
#define FOV 5                // Angle de vue des Boids (rad)
// Rayons des règles d'interaction (px)
#define R_DISTANCING 2
#define R_ALIGNMENT 5
#define R_COHESINON 8
// Poids des règles d'interaction
#define WEIGHT_DISTANCING 0.05f
#define WEIGHT_ALIGNMENT 0.05f
#define WEIGHT_COHESION 0.0005f

#define THREE_PI_OVER_FOUR (M_PIf * 0.75f)

Simulation::Simulation(int envWidth_, int envHeight_, int timeStep_)
    : envWidth(envWidth_), envHeight(envHeight_), timeStep(timeStep_), paused(false) {

    boids.speed = SPEED;
    boids.angVelocity = ANG_V;
    boids.halvedFov = FOV / 2.0f;
    boids.timeStep = timeStep / 1000.0f;

    boids.rDistancingSquared = R_DISTANCING * R_DISTANCING;
    boids.rAlignmentSquared = R_ALIGNMENT * R_ALIGNMENT;
    boids.rCohesionSquared = R_COHESINON * R_COHESINON;

    // Création d'une image de la taille de la simulation
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
}

// Lance la Simulation
void Simulation::run() {
    omp_set_num_threads(omp_get_max_threads()); // Utilise tous les threads disponibles
    std::cout << "Nombre de threads (CPU) pour l'affichage : " << omp_get_max_threads() << std::endl;
    std::vector<float> t; float total = 0.0f; long int it = 0;
    // Initialiser des boids avec des positions aléatoires
    initializeBoidsRandomly(NUM_BOIDS);
    
    // Allouer la mémoire dans la GPU
    allocateBoidDataOnGPU(boids);
// début boucle
    while (true)
    {
        // Gestion des entrées clavier
        int key = cv::waitKey(1);
        if (key != -1) {

        for (int i = 0; i < t.size(); i++) {
            total += t[i];
        }
        std::cout << "avg " << total / it << std::endl;
        handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée

        }
        // Si en pause, ne pas mettre à jour la simulation
        if (paused) continue;

        // Copier les tableaux dans la GPU
        copyBoidDataToGPU(boids);

        auto start = std::chrono::high_resolution_clock::now(); // démarrage du chronomètre
        // Appeler le kernel CUDA
        updateBoidsCUDA(
            boids.d_positionsX, boids.d_positionsY, boids.d_orientations, boids.d_interations,
            boids.positionsX.size(), envWidth, envHeight, boids.speed, boids.angVelocity, boids.timeStep,
            boids.halvedFov, boids.rDistancingSquared, boids.rAlignmentSquared, boids.rCohesionSquared,
            WEIGHT_DISTANCING, WEIGHT_ALIGNMENT, WEIGHT_COHESION
        );
        auto end = std::chrono::high_resolution_clock::now(); // fin du chronomètre
        std::chrono::duration<float, std::milli> duration = end - start; // calcul de la durée en millisecondes
        t.push_back(duration.count()); it++;
        std::cout << it << " " << duration.count() << " ms" << std::endl; // affichage du temps en millisecondes

        // Récupérer les tableaux dans le CPU
        copyBoidDataToCPU(boids);

       updateDisplay();
    }
// fin boucle
    freeBoidDataOnGPU(boids);
}

// Méthode pour initialiser les boids de manière aléatoire
void Simulation::initializeBoidsRandomly(int numBoids) {
    // Création d'un moteur aléatoire avec une graine unique
    std::random_device rd;  // Génére une graine à partir de l'environnement
    std::mt19937 gen(rd()); // Mersenne Twister : générateur de nombres pseudo-aléatoires
    std::uniform_real_distribution<> xDist(0, envWidth);
    std::uniform_real_distribution<> yDist(0, envHeight);
    std::uniform_real_distribution<> thetaDist(0, 2 * M_PIf);
    std::uniform_real_distribution<> offsetDist(0, rand());
    float offsetTheta = Types::customMod(offsetDist(gen), 2 * M_PIf);
    
    for (int i = 0; i < numBoids; ++i) {
        float newX = xDist(gen);  // Position x aléatoire
        float newY = yDist(gen);  // Position y aléatoire
        float newTheta = thetaDist(gen) + offsetTheta;  // Orientation aléatoire
        addBoid(newX, newY, newTheta);
    }
}

// Méthode pour ajouter un boid à la simulation
void Simulation::addBoid(float x, float y, float theta) {
    // Ajouter au CPU
    boids.positionsX.push_back(x);
    boids.positionsY.push_back(y);
    boids.orientations.push_back(theta);
    boids.interactions.push_back(Types::Interaction::NONE);
}

// Méthode pour supprimer un boid de la simulation
void Simulation::removeBoid(int id) {
    boids.positionsX.erase(boids.positionsX.begin() + id);
    boids.positionsY.erase(boids.positionsY.begin() + id);
    boids.orientations.erase(boids.orientations.begin() + id);
    boids.interactions.erase(boids.interactions.begin() + id);
}

// Réinitialiser la simulation
void Simulation::reset() {
    boids.positionsX.clear();
    boids.positionsY.clear();
    boids.orientations.clear();
    boids.interactions.clear();
    freeBoidDataOnGPU(boids);
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
            initializeBoidsRandomly(1);
            reallocateIfNecessary(boids);
            std::cout << "Boid ajouté." << std::endl;
            break;
        case '-': // Supprimer un boid
            removeBoid(boids.positionsX.size());
            reallocateIfNecessary(boids);
            std::cout << "Boid supprimé." << std::endl;
            break;
        case 27: // Échapper (ESC) pour quitter
            freeBoidDataOnGPU(boids);
            std::cout << "Simulation terminée." << std::endl;
            exit(0);
    }
}

// Met à jour tous les boids et affiche la simulation
void Simulation::updateDisplay() const {
    // Effacer l'image précédente
    cv::Mat image = cv::Mat::zeros(envHeight, envWidth, CV_8UC3);
    
    // Mettre à jour chaque boid
    #pragma omp parallel for
    for (int id = 0; id < boids.positionsX.size(); ++id) {
        displayBoid(image, id); // Afficher le boid dans l'image
    }
    
    // Afficher l'image dans une fenêtre OpenCV
    cv::namedWindow("Simulation", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Simulation", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow("Simulation", image);
}

// Affiche chaque boid avec une couleur selon son interaction
void Simulation::displayBoid(cv::Mat& image, int id) const {
    // Déterminer la couleur en fonction de l'interaction
    cv::Scalar color;
    Types::Interaction currentInteraction = boids.interactions[id];
    switch (currentInteraction) {
        case Types::Interaction::DISTANCING: color = cv::Scalar(0, 0, 255);   break;
        case Types::Interaction::ALIGNMENT:  color = cv::Scalar(0, 255, 0);   break;
        case Types::Interaction::COHESION:   color = cv::Scalar(255, 0, 0);   break;
        case Types::Interaction::NONE:       color = cv::Scalar(127, 127, 0); break;
    }

    // Dessiner le boid sous forme de triangle isocèle
    float x = boids.positionsX[id];
    float y = boids.positionsY[id];
    float theta = boids.orientations[id];
    
    // Calcul et dessin en une "pseudo-ligne"
    cv::Point points[3] = {
        cv::Point(x + cos(theta), y + 0.5f * sin(theta)),
        cv::Point(x + 0.5f * cos(theta + THREE_PI_OVER_FOUR), y + 0.5f * sin(theta + THREE_PI_OVER_FOUR)),
        cv::Point(x + 0.5f * cos(theta - THREE_PI_OVER_FOUR), y + 0.5f * sin(theta - THREE_PI_OVER_FOUR))
    };
    //cv::circle(image, cv::Point(x, y), 1, color, cv::FILLED);
    cv::fillPoly(image, std::vector<cv::Point>{points, points + 3}, color);
}




// Méthode pour basculer l'état de pause
void Simulation::togglePause() {
    paused = !paused;
}

Simulation::~Simulation() {
    reset();
    cv::destroyAllWindows();
}

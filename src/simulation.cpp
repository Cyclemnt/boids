#include "../include/simulation.hpp"
#include "../include/gpu_utils.cuh"
#include "../include/constants.hpp"
#include <omp.h>
#include <random>
#include <chrono>

Simulation::Simulation()
    : paused(false), d_x(nullptr), d_y(nullptr), d_theta(nullptr), d_interaction(nullptr) {}

// Lance la Simulation
void Simulation::run() {
    // Calculer les dimensions des cellules et leur nombre
    const float inverseCellWidth = 1.0 / findMinDivisor(ENV_WIDTH, R_COHESION);
    const float inverseCellHeight = 1.0 / findMinDivisor(ENV_HEIGHT, R_COHESION);
    const int numCellWidth = ENV_WIDTH * inverseCellWidth;
    const int numCellHeight = ENV_HEIGHT * inverseCellHeight;
    const int numCells = numCellWidth * numCellHeight;
    cellCount.resize(numCells + 1);

    omp_set_num_threads(omp_get_max_threads()); // Utilise tous les threads disponibles (pour l'affichage)
    // Initialiser des boids avec des positions aléatoires
    initializeBoidsRandomly(NUM_BOIDS);
    
    // Allouer la mémoire dans la GPU et copier les données
    allocateBoidDataOnGPU();
    copyBoidDataToGPU();
//double temps = 0;
    for (int i = 0; i < 1000; ++i) {
        int key = cv::waitKey(1); // Gestion des entrées clavier
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        if (paused) continue; // Si en pause, ne pas mettre à jour la simulation
//auto debut = std::chrono::high_resolution_clock::now();
        // Appeler le kernel CUDA
        updateBoidsCUDA(d_x, d_y, d_theta, d_interaction, x.size(), d_cellCount, d_particleMap, numCells, numCellWidth, numCellHeight, inverseCellWidth, inverseCellHeight);
//auto fin = std::chrono::high_resolution_clock::now();
//std::chrono::duration<double, std::milli> duree = fin - debut;
//temps += duree.count();
        // Récupérer les tableaux dans le CPU et afficher
        copyBoidDataToCPU();
        updateDisplay();
    }
//std::cout << "Temps d'exécution moyen : " << temps / 1000.0 << std::endl;

    freeBoidDataOnGPU();
}

// Méthode pour initialiser les boids de manière aléatoire
void Simulation::initializeBoidsRandomly(int numBoids) {
    // Création d'un moteur aléatoire avec une graine unique
    std::random_device rd;  // Génére une graine à partir de l'environnement
    std::mt19937 gen(rd()); // Mersenne Twister : générateur de nombres pseudo-aléatoires
    std::uniform_real_distribution<> xDist(0, ENV_WIDTH);
    std::uniform_real_distribution<> yDist(0, ENV_HEIGHT);
    std::uniform_real_distribution<> thetaDist(0, 2 * M_PIf);
    std::uniform_real_distribution<> offsetDist(0, rand());
    float offsetTheta = fmodf(offsetDist(gen), 2 * M_PIf);
    
    for (int i = 0; i < numBoids; ++i) {
        float newX = xDist(gen);  // Position x aléatoire
        float newY = yDist(gen);  // Position y aléatoire
        float newTheta = thetaDist(gen) + offsetTheta;  // Orientation aléatoire
        addBoid(newX, newY, newTheta);
    }
}

// Fonction pour trouver le diviseur min d'un nombre supérieur ou égal à un certain seuil
int Simulation::findMinDivisor(int number, int minSize) {
    for (int i = minSize; i <= number; ++i) {
        if (number % i == 0) return i;
    } // Voir si arrondi possible
    return -1;
}

// Met à jour tous les boids et affiche la simulation
void Simulation::updateDisplay() const {
    // Effacer l'image précédente
    cv::Mat image = cv::Mat::zeros(ENV_HEIGHT, ENV_WIDTH, CV_8UC3);
    
    // Mettre à jour chaque boid
    #pragma omp parallel for
    for (int id = 0; id < x.size(); ++id) {
        displayBoid(image, id); // Afficher le boid dans l'image
    }
    
    // Afficher l'image dans une fenêtre OpenCV
    cv::namedWindow("Simulation", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Simulation", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow("Simulation", image);
}

// Affiche chaque boid avec une couleur selon son interaction
inline void Simulation::displayBoid(cv::Mat& image, int id) const {
    // Déterminer la couleur en fonction de l'interaction
    cv::Scalar color;
    int currentInteraction = interaction[id];
    switch (currentInteraction) {
        case 1: color = cv::Scalar(0, 0, 255);   break;
        case 2: color = cv::Scalar(0, 255, 0);   break;
        case 3: color = cv::Scalar(255, 0, 0);   break;
        case 0: color = cv::Scalar(127, 127, 0); break;
    }

    // Dessiner le boid
    float posX = x[id];
    float posY = y[id];
    float angle = theta[id];
    // Calcul et dessin en une ligne
    cv::line(image, cv::Point(posX, posY), cv::Point(posX + cosf(angle), posY + sinf(angle)), color, 1, 8, 0);
}

// Méthode pour ajouter un boid à la simulation
inline void Simulation::addBoid(float x_, float y_, float theta_) {
    // Ajouter au CPU
    x.push_back(x_);
    y.push_back(y_);
    theta.push_back(theta_);
    interaction.push_back(0);
    particleMap.push_back(0);
}

// Méthode pour supprimer un boid de la simulation
void Simulation::removeBoid(int id) {
    x.erase(x.begin() + id);
    y.erase(y.begin() + id);
    theta.erase(theta.begin() + id);
    interaction.erase(interaction.begin() + id);
    particleMap.erase(particleMap.begin()); // Peu importe la case supprimée
}

// Réinitialiser la simulation
void Simulation::reset() {
    x.clear();
    y.clear();
    theta.clear();
    interaction.clear();
    freeBoidDataOnGPU();
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
            reallocate();
            std::cout << "Boid ajouté." << std::endl;
            break;
        case '-': // Supprimer un boid
            removeBoid(x.size());
            reallocate();
            std::cout << "Boid supprimé." << std::endl;
            break;
        case 27: // Échapper (ESC) pour quitter
            freeBoidDataOnGPU();
            std::cout << "Simulation terminée." << std::endl;
            exit(0);
    }
}

// Méthode pour basculer l'état de pause
void Simulation::togglePause() {
    paused = !paused;
}

//FONCTIONS UTILES CUDA
// Alloue la mémoire GPU pour les données des Boids
void Simulation::allocateBoidDataOnGPU() {
    size_t dataSizef = x.size() * sizeof(float);
    size_t dataSizei = x.size() * sizeof(int);
    size_t cellCountSize = cellCount.size() * sizeof(int);

    // Allocation pour chaque vecteur
    cudaMalloc(&d_x, dataSizef);
    cudaMalloc(&d_y, dataSizef);
    cudaMalloc(&d_theta, dataSizef);
    cudaMalloc(&d_interaction, dataSizei);
    cudaMalloc(&d_cellCount, cellCountSize);
    cudaMalloc(&d_particleMap, dataSizei);
}

// Libère la mémoire GPU
void Simulation::freeBoidDataOnGPU() {
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_theta);
    cudaFree(d_interaction);
    cudaFree(d_cellCount);
    cudaFree(d_particleMap);
}

// Transfère les données CPU -> GPU
void Simulation::copyBoidDataToGPU() {
    size_t dataSize = x.size() * sizeof(float);

    cudaMemcpy(d_x, x.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_theta, theta.data(), dataSize, cudaMemcpyHostToDevice);
}

// Transfère les données GPU -> CPU
void Simulation::copyBoidDataToCPU() {
    size_t dataSizef = x.size() * sizeof(float);
    size_t dataSizei = x.size() * sizeof(int);

    cudaMemcpy(x.data(), d_x, dataSizef, cudaMemcpyDeviceToHost);
    cudaMemcpy(y.data(), d_y, dataSizef, cudaMemcpyDeviceToHost);
    cudaMemcpy(theta.data(), d_theta, dataSizef, cudaMemcpyDeviceToHost);
    cudaMemcpy(interaction.data(), d_interaction, dataSizei, cudaMemcpyDeviceToHost);
}

// Réallocation si la taille change
inline void Simulation::reallocate() {
    freeBoidDataOnGPU();
    allocateBoidDataOnGPU();
    copyBoidDataToGPU();
}




Simulation::~Simulation() {
    reset();
    cv::destroyAllWindows();
}

#include "../include/simulation.hpp"
#include "../include/gpu_utils.cuh"
#include "../include/constants.hpp"
#include <opencv2/opencv.hpp>
#include <random>
#include <chrono>

// Constructeur
Simulation::Simulation()
    : paused(false), d_x(nullptr), d_y(nullptr), d_theta(nullptr), d_boidMap(nullptr), d_cellCount(nullptr), d_image(nullptr), running(true),
      inverseCellWidth(1.0 / findMinDivisor(ENV_WIDTH, R_COHESION)), inverseCellHeight(1.0 / findMinDivisor(ENV_HEIGHT, R_COHESION)),
      numCellWidth(ENV_WIDTH * inverseCellWidth), numCellHeight(ENV_HEIGHT * inverseCellHeight), numCells(ENV_WIDTH * inverseCellWidth * ENV_HEIGHT * inverseCellHeight) {
    cellCount.resize(numCells + 1);
}

// Lancer la simulation
void Simulation::run() {
    // Initialiser des boids avec des positions aléatoires
    initializeBoidsRandomly(NUM_BOIDS);
    
    // Allouer la mémoire GPU et copier les données
    allocateBoidDataOnGPU();
    copyBoidDataToGPU();

    // Boucle principale
    while (running) {
        int key = cv::waitKey(1); // Récupérer les entrées clavier
        if (key != -1) handleKeyPress(key); // Si une touche a été pressée, traiter l'entrée
        if (paused) continue; // Si en pause, ne pas mettre à jour la simulation

        // Appeler le kernel CUDA
        updateBoidsCUDA(d_x, d_y, d_theta, d_image, x.size(), d_cellCount, d_boidMap, numCells, numCellWidth, numCellHeight, inverseCellWidth, inverseCellHeight);
        // Afficher
        updateDisplay();
    }
    
    // Libérer la mémoire GPU
    freeBoidDataOnGPU();
}

// Initialiser les boids de manière aléatoire
void Simulation::initializeBoidsRandomly(int numBoids) {
    // Création d'un moteur aléatoire avec une graine unique
    std::random_device rd;  // Génére une graine à partir de l'environnement
    std::mt19937 gen(rd()); // Mersenne Twister : générateur de nombres pseudo-aléatoires
    std::uniform_real_distribution<> xDist(0, ENV_WIDTH);
    std::uniform_real_distribution<> yDist(0, ENV_HEIGHT);
    std::uniform_real_distribution<> thetaDist(0, 2 * M_PIf);
    std::uniform_real_distribution<> offsetDist(0, rand());
    
    for (int i = 0; i < numBoids; ++i) {
        float newX = xDist(gen); // Position x aléatoire
        float newY = yDist(gen); // Position y aléatoire
        float newTheta = thetaDist(gen); // Orientation aléatoire
        addBoid(newX, newY, newTheta);
    }
}

 // Trouver le plus petit diviseur commun
int Simulation::findMinDivisor(int number, int minSize) {
    for (int i = minSize; i <= number; ++i) {
        if (number % i == 0) return i;
    }
    return -1;
}

// Afficher la simulation
inline void Simulation::updateDisplay() const {
    cv::Mat image(ENV_HEIGHT, ENV_WIDTH, CV_8UC3); // Créer l'image
    cudaMemcpy(image.data, d_image, ENV_WIDTH * ENV_HEIGHT * 3, cudaMemcpyDeviceToHost); // Récupérer l'image du GPU
    // Afficher
    cv::namedWindow("Simulation", cv::WINDOW_NORMAL);
    cv::setWindowProperty("Simulation", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::imshow("Simulation", image);
}

// Ajouter un boid à la simulation
inline void Simulation::addBoid(float x_, float y_, float theta_) {
    x.push_back(x_);
    y.push_back(y_);
    theta.push_back(theta_);
    boidMap.push_back(0);
}

// Supprimer un boid de la simulation
inline void Simulation::removeBoid(int id) {
    if (x.empty()) return;
    x.erase(x.begin() + id);
    y.erase(y.begin() + id);
    theta.erase(theta.begin() + id);
    boidMap.erase(boidMap.begin()); // Peu importe la case supprimée
}

// Réinitialiser la simulation
inline void Simulation::reset() {
    x.clear();
    y.clear();
    theta.clear();
    boidMap.clear();
}

// Gérer les touches
inline void Simulation::handleKeyPress(int key) {
    switch (key) {
        case 'p': // Pause ou reprise
            togglePause();
            std::cout << (paused ? "Simulation en pause." : "Simulation reprise.") << std::endl;
            break;
        case 'r': // Réinitialiser
            reset();
            reallocate();
            std::cout << "Simulation réinitialisée." << std::endl;
            break;
        case '+': // Ajouter un boid
            copyBoidDataToCPU();
            initializeBoidsRandomly(1024);
            reallocate();
            std::cout << "1024 boids ajoutés." << std::endl;
            break;
        case '-': // Supprimer un boid
            copyBoidDataToCPU();
            for (int i = 0; i < 1024; i++) removeBoid(x.size());
            reallocate();
            std::cout << "1024 boids supprimés." << std::endl;
            break;
        case 27: // Échapper (ESC) pour quitter
            running = false;
            freeBoidDataOnGPU();
            std::cout << "Simulation terminée." << std::endl;
            exit(0);
    }
}

// Basculer l'état de pause
inline void Simulation::togglePause() {
    paused = !paused;
}

//FONCTIONS UTILES CUDA
// Allouer la mémoire GPU
inline void Simulation::allocateBoidDataOnGPU() {
    size_t dataSizef = x.size() * sizeof(float);
    size_t dataSizei = x.size() * sizeof(int);
    size_t cellCountSize = cellCount.size() * sizeof(int);

    // Allocation pour chaque vecteur
    cudaMalloc(&d_x, dataSizef);
    cudaMalloc(&d_y, dataSizef);
    cudaMalloc(&d_theta, dataSizef);
    cudaMalloc(&d_cellCount, cellCountSize);
    cudaMalloc(&d_boidMap, dataSizei);
    cudaMalloc(&d_image, ENV_WIDTH * ENV_HEIGHT * 3 * sizeof(unsigned char));  // 3 canaux pour RGB
}

// Libérer la mémoire GPU
inline void Simulation::freeBoidDataOnGPU() {
    cudaFree(d_x);
    cudaFree(d_y);
    cudaFree(d_theta);
    cudaFree(d_cellCount);
    cudaFree(d_boidMap);
    cudaFree(d_image);
}

// Transférer les données CPU -> GPU
inline void Simulation::copyBoidDataToGPU() {
    size_t dataSize = x.size() * sizeof(float);

    cudaMemcpy(d_x, x.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_y, y.data(), dataSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_theta, theta.data(), dataSize, cudaMemcpyHostToDevice);
}

// Transférer les données GPU -> CPU
inline void Simulation::copyBoidDataToCPU() {
    size_t dataSizef = x.size() * sizeof(float);
    size_t dataSizei = x.size() * sizeof(int);

    cudaMemcpy(x.data(), d_x, dataSizef, cudaMemcpyDeviceToHost);
    cudaMemcpy(y.data(), d_y, dataSizef, cudaMemcpyDeviceToHost);
    cudaMemcpy(theta.data(), d_theta, dataSizef, cudaMemcpyDeviceToHost);
}

// Allouer à nouveau
inline void Simulation::reallocate() {
    freeBoidDataOnGPU();
    allocateBoidDataOnGPU();
    copyBoidDataToGPU();
}

// Destructeur
Simulation::~Simulation() {
    running = false;
    reset();
    freeBoidDataOnGPU();
    cv::destroyAllWindows();
}

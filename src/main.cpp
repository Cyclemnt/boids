#define TIMESTEP 16
#include "../include/simulation.hpp"

int main() {
    Simulation sim(1900, 1000, TIMESTEP); //Définition de la taille de la fenêtre de travail 

    sim.run();
    
    return 0;
}
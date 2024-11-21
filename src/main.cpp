#define TIMESTEP 16
#include "../include/simulation.hpp"

int main() {
    Simulation sim(1920/3, 1080/3, TIMESTEP);

    sim.run();
    
    return 0;
}
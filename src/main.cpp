#define TIMESTEP 32
#include "../include/simulation.hpp"

int main() {
    Simulation sim(1920, 1200, TIMESTEP);

    sim.run();
    
    return 0;
}
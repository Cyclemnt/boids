#define TIMESTEP 16
#include "../include/simulation.hpp"

int main() {
    Simulation sim(1900 / 2, 1000 / 2, TIMESTEP);

    sim.run();
    
    return 0;
}
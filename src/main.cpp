#define TIMESTEP 16
#include "../include/simulation.hpp"

int main() {
    Simulation sim(1900, 1000, TIMESTEP);

    sim.run();
    
    return 0;
}
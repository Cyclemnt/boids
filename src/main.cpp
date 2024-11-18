
#define TIMESTEP 128
#include "../include/simulation.hpp"
#include <iostream>

int main() {
    Simulation sim(1920,1800, TIMESTEP);

    sim.run();
    
    return 0;
}
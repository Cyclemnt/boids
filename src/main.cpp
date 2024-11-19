#define TIMESTEP 32
#include "../include/simulation.hpp"
#include <iostream>

int main() {
    Simulation sim(1900, 1000, TIMESTEP);

    sim.run();
    
    return 0;
}
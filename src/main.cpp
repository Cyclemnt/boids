#include "../include/simulation.hpp"
#include <iostream>

#define TIMESTEP 32

int main() {
    Simulation sim(100, 100, TIMESTEP);

    sim.run();
    
    return 0;
}
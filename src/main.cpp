#define TIMESTEP 32
#include "../include/simulation.hpp"
#include <iostream>

int main() {
    Simulation sim(2000, 2000, TIMESTEP);

    sim.run();
    
    return 0;
}




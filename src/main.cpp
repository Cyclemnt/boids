
#define TIMESTEP 32
#include "../include/simulation.hpp"
#include <iostream>

int main() {
    Simulation sim(100,100, TIMESTEP);

    sim.run();
    
    return 0;
}




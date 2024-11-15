
#define TIMESTEP 5
#include "../include/simulation.hpp"
#include <iostream>

int main() {
    Simulation sim(150,150, TIMESTEP);

    sim.run();
    
    return 0;
}
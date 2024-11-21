#define TIMESTEP 16
#include "../include/simulation.hpp"

int main() {
    Simulation sim(1900/3, 1000/3, TIMESTEP);

    sim.run();
    
    return 0;
}
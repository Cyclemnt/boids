#define TIMESTEP 16
#include "../include/simulation.hpp"

int main() {
    Simulation sim(2 * 1920, 2 * 1200, TIMESTEP);

    sim.run();
    
    return 0;
}
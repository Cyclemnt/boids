#define TIMESTEP 16
#include "../include/simulation.hpp"

int main() {
    Simulation sim(1920, 1080, TIMESTEP);

    sim.run();
    
    return 0;
}
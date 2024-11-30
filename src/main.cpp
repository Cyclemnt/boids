#define TIMESTEP 16
#include "../include/simulation.hpp"

int main() {
    Simulation sim(1920/1, 1080/1, TIMESTEP);

    sim.run();
    
    return 0;
}
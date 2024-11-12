#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include"boid.hpp"
#include <vector>

class Simulation {
    private : 
        double width;
        double height;
        std::vector<Boid*> boids;
    public :
        Simulation(double width_, double height_);

        void addBoid();
        void removeBoid();
        void reset();
        void update();
        void togglePause();
        bool isPaused() const ;
        void sendInfoToBoids();

        ~Simulation();
};

#endif // SIMULATION_HPP
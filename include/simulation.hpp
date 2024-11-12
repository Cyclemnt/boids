#include<vector>
#include"boid.hpp"

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
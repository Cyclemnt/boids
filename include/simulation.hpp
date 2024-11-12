#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "boid.hpp"
#include <vector>
#include <opencv2/opencv.hpp>

class Simulation {
    private : 
        double width;
        double height;
        std::vector<Boid*> boids;
    public :
        Simulation(double width_, double height_);

        void addBoid(vPose pose, int fov, double maxSpeed, double maxAngVelocity);
        void removeBoid();
        void reset();
        void update();
        void displayBoid(cv::Mat* image, const Boid* boid);
        void togglePause();
        bool isPaused() const ;
        void sendInfoToBoids();

        ~Simulation();
};

#endif // SIMULATION_HPP
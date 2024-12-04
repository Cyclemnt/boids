#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

// Nombre de boids initiaux
#define NUM_BOIDS 4 * 65536
// Vitesse des boids
#define SPEED 70
#define ANG_V 2 * M_PIf
// Dimensions de l'environnement
#define ENV_WIDTH (2 * 1920)
#define ENV_HEIGHT (2 * 1200)
// Rayons d'interactions
#define R_DISTANCING 4.0f
#define R_ALIGNMENT 10.0f
#define R_COHESION 16.0f
// Fielf of view
#define FOV 5.0f
// Poids des interactions
#define WEIGHT_DISTANCING 0.05f
#define WEIGHT_ALIGNMENT 0.05f
#define WEIGHT_COHESION 0.0005f
// Temps entre chaque calculs
#define TIME_STEP 0.03f

#endif // CONSTANTS_HPP

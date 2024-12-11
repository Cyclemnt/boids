#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

// Nombre de boids initiaux
#define NUM_BOIDS 524288/2
// Vitesse des boids
#define SPEED 40.0f
#define ANG_V M_PIf
// Dimensions de l'environnement
#define ENV_WIDTH (1920)
#define ENV_HEIGHT (1200)
// Rayons d'interactions
#define R_DISTANCING 1.0f
#define R_ALIGNMENT 5.0f
#define R_COHESION 5.0f
// Fielf of view
#define FOV 5.0f
// Poids des interactions
#define WEIGHT_DISTANCING 0.5f
#define WEIGHT_ALIGNMENT 0.05f
#define WEIGHT_COHESION 0.005f
// Temps entre chaque calculs
#define TIME_STEP 0.012f

#endif // CONSTANTS_HPP
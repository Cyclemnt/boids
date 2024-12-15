#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// Nombre de boids initiaux
#define NUM_BOIDS 524288/2
// Vitesse des boids
#define SPEED 40.0f             // Translation (px/s)
#define ANG_V M_PIf             // Rotation (rad/s)
// Dimensions de l'environnement
#define ENV_WIDTH (1920)        // Largeur (px)
#define ENV_HEIGHT (1200)       // Hauteur (px)
// Rayons d'interactions
#define R_DISTANCING 2.0f       // Distanciation (px)
#define R_ALIGNMENT 9.0f        // Alignement (px)
#define R_COHESION 9.0f         // Cohésion (px)
// Field of view
#define FOV 5.0f                // Champ de vision (rad)
// Poids des interactions
#define WEIGHT_DISTANCING 0.5f  // Poids de distanciation (1)
#define WEIGHT_ALIGNMENT 0.05f  // Poids d'alignement (1)
#define WEIGHT_COHESION 0.005f  // Poids de cohésion (1)
// Temps entre chaque état de simulation (affichage à vitesse maximale)
#define TIME_STEP 0.015f        // Ajuster en fonction du matériel et du nombre de boids
                                // Recommandé pour RTX3060 : 
                                // NUM_BOIDS    TIME_STEP
                                //   250 000    0.015f
                                //   500 000    0.030f
                                // 1 000 000    0.060f
                                // 2 000 000    0.120f

#endif // CONSTANTS_HPP

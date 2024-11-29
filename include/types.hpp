#ifndef TYPES_HPP
#define TYPES_HPP

#include <cmath>

namespace Types
{
    struct vPose {
        double x, y, theta;

        // Surcharge de l'opérateur + pour l'addition de deux vPose
        vPose operator+(const vPose& other) const {
            return {x + other.x, y + other.y, theta + other.theta};
        }
        // Surcharge de l'opérateur - pour la soustraction de deux vPose
        vPose operator-(const vPose& other) const {
            return {x - other.x, y - other.y, theta - other.theta};
        }
        // Surcharge de l'opérateur / pour la division par un scalaire
        vPose operator/(double scalar) const {
            return {x / scalar, y / scalar, theta / scalar};
        }
        // Surcharge de l'opérateur == pour comparer deux vPose
        bool operator==(const vPose& other) const {
            return (x == other.x) && (y == other.y) && (theta == other.theta);
        }
        // Surcharge de l'opérateur != comme complément de ==
        bool operator!=(const vPose& other) const {
            return !(*this == other);
        }
    };

    enum class Interaction { DISTANCING, ALIGNMENT, COHESION, FOLLOW,NONE };

    // Modulo customisé
    inline double customMod(double a, double n) {return fmod(fmod(a, n) + n, n);}
} // namespace Types

#endif // TYPES_HPP
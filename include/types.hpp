#ifndef TYPES_HPP
#define TYPES_HPP

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
    };

    enum class Interaction { DISTANCING, ALIGNMENT, COHESION,NOTHING };
} // namespace Types

#endif // TYPES_HPP
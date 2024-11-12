#ifndef TYPES_HPP
#define TYPES_HPP

namespace Types
{
    struct vPose {
        double x, y, theta;
    };

    enum class Interaction { DISTANCING, ALIGNMENT, COHESION };
} // namespace Types

#endif // TYPES_HPP
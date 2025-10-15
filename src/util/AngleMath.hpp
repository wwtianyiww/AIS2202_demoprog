#ifndef DEMOPROG1_ANGLEMATH_HPP
#define DEMOPROG1_ANGLEMATH_HPP
#include <cmath>
namespace AngleMath {
    float rawToDegrees(uint16_t raw);
    float degreesToRadians(float degrees);
    float radiansToDegrees(float radians);
    float angleError(float target, float measured);
    float wrapAngle(float angle);
}

#endif //DEMOPROG1_ANGLEMATH_HPP
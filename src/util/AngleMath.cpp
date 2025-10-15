#include "AngleMath.hpp"

namespace AngleMath {

    // Convert AS5600 raw value (0-4095) to degrees (0-360)
    float rawToDegrees(uint16_t raw) {
        return raw * (360.0f / 4096.0f);
    }

    // Convert degrees to radians
    float degreesToRadians(float degrees) {
        return degrees * (M_PI / 180.0f);
    }

    // Convert radians to degrees
    float radiansToDegrees(float radians) {
        return radians * (180.0f / M_PI);
    }

    // Calculate shortest-path wrapped error [-180, 180]
    float angleError(float target, float measured) {
        float e = target - measured;
        while (e >  180.0f) e -= 360.0f;
        while (e < -180.0f) e += 360.0f;
        return e;
    }

    // Wrap angle to [0, 360) range
    float wrapAngle(float angle) {
        angle = fmodf(angle, 360.0f);
        if (angle < 0.0f) {
            angle += 360.0f;
        }
        return angle;
    }

}
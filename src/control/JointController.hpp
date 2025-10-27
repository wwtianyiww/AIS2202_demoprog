#ifndef DEMOPROG1_JOINTCONTROLLER_HPP
#define DEMOPROG1_JOINTCONTROLLER_HPP
#include "hardware/Joint.hpp"
#include "DeadBandController.hpp"
#include "util/AngleMath.hpp"

class JointController {
public:
    JointController(Joint& joint, DeadbandController& controller);
    void setTarget(float targetAngle);
    void update(float dt);
    float getAngle();
    uint16_t getRawAngle();
    float getError();
    float getCurrent();
private:
    Joint& joint_;
    DeadbandController& controller_;
    float targetAngle_;
};
#endif //DEMOPROG1_JOINTCONTROLLER_HPP
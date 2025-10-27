#ifndef DEMOPROG1_ROBOTARM_HPP
#define DEMOPROG1_ROBOTARM_HPP
#include "control/JointController.hpp"
#include "StateMonitor.hpp"

class RobotArm {
public:
    RobotArm(const JointController& shoulderController,
                   const JointController& wristController,
                   const StateMonitor& monitor);
    void initialize();
    void update(float dt);
    void setShoulderTarget(float angle);
    void setWristTarget(float angle);
    void setTargets(float shoulder, float wrist);
    void homeAll();
    float getShoulderAngle();
    float getWristAngle();
    float getShoulderError();
    float getWristError();
    float getShoulderCurrent();
    float getWristCurrent();

    uint16_t getRawShoulder();

    uint16_t getRawWrist();

    void printStatus();

private:
    JointController shoulderController_;
    JointController wristController_;
    StateMonitor monitor_;
};
#endif //DEMOPROG1_ROBOTARM_HPP
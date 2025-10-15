#include "JointController.hpp"

JointController::JointController(Joint& joint, DeadbandController& controller)
    : joint_(joint), controller_(controller), targetAngle_(0.0f) {
}

void JointController::setTarget(float targetAngle) {
    targetAngle_ = targetAngle;
}

void JointController::update(float dt) {
    // Read current angle from the joint
    float currentAngle = joint_.getAngle();

    // Calculate angle error using shortest path
    float error = AngleMath::angleError(targetAngle_, currentAngle);

    // Compute control signal using the deadband controller
    bool shouldStop = false;
    float controlSignal = controller_.compute(error, dt, shouldStop);

    // Drive or stop the joint based on deadband logic
    if (shouldStop) {
        joint_.stop();
    } else {
        joint_.drive(controlSignal);
    }
}

float JointController::getAngle() {
    return joint_.getAngle();
}

float JointController::getError() {
    float currentAngle = joint_.getAngle();
    return AngleMath::angleError(targetAngle_, currentAngle);
}

float JointController::getCurrent() {
    return joint_.getCurrent();
}
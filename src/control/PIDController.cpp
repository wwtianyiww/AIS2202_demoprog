#include "PIDController.hpp"

PIDController::PIDController(float kP, float kI, float kD, float maxIntegral) :
kP_(kP), kI_(kI), kD_(kD), maxIntegral_(maxIntegral), integral_(0.0f), previousError_(0.0f) {

}

float PIDController::compute(float error, float dt) {
    // Update integral with anti-windup
    integral_ += error * dt;
    if (integral_ > maxIntegral_) integral_ = maxIntegral_;
    if (integral_ < -maxIntegral_) integral_ = -maxIntegral_;
    // Calculate derivative (with protection against division by zero)
    float derivative = (dt > 1e-6f) ? (error - previousError_) / dt : 0.0f;
    // Store current error for next iteration
    previousError_ = error;
    // Calculate PID output
    return kP_ * error + kI_ * integral_ + kD_ * derivative;
}


void PIDController::reset() {
    integral_ = 0.0f;
    previousError_ = 0.0f;
}

void PIDController::setGains(float kP, float kI, float kD) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
}

void PIDController::setIntegralLimit(float maxIntegral) {
    maxIntegral_ = maxIntegral;
}



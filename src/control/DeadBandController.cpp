#include "DeadBandController.hpp"
DeadbandController::DeadbandController(PIDController& pid, float threshold)
    : pid_(pid), threshold_(threshold) {
}

void DeadbandController::setThreshold(float threshold) {
    threshold_ = threshold;
}

float DeadbandController::compute(float error, float dt, bool& shouldStop) {
    // Check if error is within deadband threshold
    if (fabsf(error) <= threshold_) {
        // Within deadband - signal to stop and return zero control signal
        shouldStop = true;
        pid_.reset();  // Reset PID integral to prevent wind-up when stopped
        return 0.0f;
    }
    
    // Outside deadband - use PID control
    shouldStop = false;
    return pid_.compute(error, dt);
}
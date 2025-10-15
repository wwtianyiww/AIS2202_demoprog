#ifndef DEMOPROG1_CONTROLCONFIG_HPP
#define DEMOPROG1_CONTROLCONFIG_HPP


namespace ControlConfig {
    // PID gains for shoulder
    constexpr float shoulderKp = 1.0f;
    constexpr float shoulderKi = 0.1f;
    constexpr float shoulderKd = 0.05f;
    constexpr float shoulderMaxIntegral = 200.0f;

    // PID gains for wrist
    constexpr float wristKp = 1.0f;
    constexpr float wristKi = 0.1f;
    constexpr float wristKd = 0.05f;
    constexpr float wristMaxIntegral = 200.0f;

    // Deadband thresholds (degrees)
    constexpr float shoulderDeadbandThreshold = 1.0f;
    constexpr float wristDeadbandThreshold = 1.0f;

}



#endif //DEMOPROG1_CONTROLCONFIG_HPP
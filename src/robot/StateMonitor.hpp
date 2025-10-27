#ifndef DEMOPROG1_STATEMONITOR_HPP
#define DEMOPROG1_STATEMONITOR_HPP
#include <Arduino.h>

class StateMonitor {
public:
    explicit StateMonitor(long baudRate = 9600);
    void initialize();
    void logJointState(const char* name, float angle, float error, float current);
    void logBothJoints(float shoulderAngle, uint16_t rawShoulder, float shoulderError, float shoulderCurrent,
                      float wristAngle, uint16_t rawWrist, float wristError, float wristCurrent);
    void logError(const char* message);
    void setVerbose(bool verbose);
private:
    long baudRate_;
    bool verbose_;
};
#endif //DEMOPROG1_STATEMONITOR_HPP
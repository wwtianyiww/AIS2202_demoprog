#ifndef DEMOPROG1_STATEMONITOR_HPP
#define DEMOPROG1_STATEMONITOR_HPP
#include <Arduino.h>

class StateMonitor {
public:
    explicit StateMonitor(long baudRate = 9600);
    void initialize();
    void logJointState(const char* name, float angle, float error, float current);
    void logBothJoints(float shoulderAngle, float shoulderError, float shoulderCurrent,
                      float wristAngle, float wristError, float wristCurrent);
    void logError(const char* message);
    void setVerbose(bool verbose);
private:
    long baudRate_;
    bool verbose_;
};
#endif //DEMOPROG1_STATEMONITOR_HPP
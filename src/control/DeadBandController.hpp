#ifndef DEMOPROG1_DEADBANDCONTROLLER_HPP
#define DEMOPROG1_DEADBANDCONTROLLER_HPP
#include "PIDController.hpp"
#include "cmath"

class DeadbandController {
public:
    DeadbandController(PIDController& pid, float threshold);
    float compute(float error, float dt, bool& shouldStop);
    void setThreshold(float threshold);
private:
    PIDController& pid_;
    float threshold_;
};
#endif //DEMOPROG1_DEADBANDCONTROLLER_HPP
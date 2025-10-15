#ifndef DEMOPROG1_PIDCONTROLLER_HPP
#define DEMOPROG1_PIDCONTROLLER_HPP
class PIDController {
public:
    PIDController(float kP, float kI, float kD, float maxIntegral = 200.0f);
    float compute(float error, float dt);
    void reset();
    void setGains(float kP, float kI, float kD);
    void setIntegralLimit(float maxIntegral);
private:
    float kP_, kI_, kD_;
    float integral_;
    float previousError_;
    float maxIntegral_;
};

#endif //DEMOPROG1_PIDCONTROLLER_HPP
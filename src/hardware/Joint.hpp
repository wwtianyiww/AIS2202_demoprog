#ifndef DEMOPROG1_JOINT_HPP
#define DEMOPROG1_JOINT_HPP
#include "Motor.hpp"
#include "AS5600Sensor.hpp"
#include "CurrentSensor.hpp"

class Joint {
public:
    Joint(Motor& motor, AS5600Sensor& sensor, CurrentSensor& current);
    void initialize();
    float getAngle();
    float getCurrent();
    void drive(float controlSignal, int pwmDuty = 50);
    void stop();
private:
    Motor& motor_;
    AS5600Sensor& sensor_;
    CurrentSensor& current_;
};

#endif //DEMOPROG1_JOINT_HPP
#ifndef DEMOPROG1_MOTOR_HPP
#define DEMOPROG1_MOTOR_HPP
#include <Arduino.h>
class Motor {
public:
    Motor(int pin1, int pin2, int pinPWM);
    void initialize();
    void driveMotor(float controlSignal, int pwmDuty = 50);
    void stopMotor();
private:
    int pin1_;
    int pin2_;
    int pwm_;
};


#endif //DEMOPROG1_MOTOR_HPP
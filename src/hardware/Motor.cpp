#include "Motor.hpp"

Motor::Motor(int pin1, int pin2, int pwm): pin1_(pin1), pin2_(pin2), pwm_(pwm) {
}


void Motor::initialize() {
    pinMode(pin1_, OUTPUT);
    pinMode(pin2_, OUTPUT);
    pinMode(pwm_, OUTPUT);

    stopMotor();
}


void Motor::stopMotor() {
    analogWrite(pwm_, 0);
    digitalWrite(pin1_, LOW);
    digitalWrite(pin2_, LOW);
}

void Motor::driveMotor(float controlSignal, int pwmDuty) {
    if (controlSignal >= 0.0f) {
        digitalWrite(pin1_, HIGH);
        digitalWrite(pin2_, LOW);
        analogWrite(pwm_, pwmDuty);
    }else {
        digitalWrite(pin1_, LOW);
        digitalWrite(pin2_,HIGH);
        analogWrite(pwm_, pwmDuty);
    }
}
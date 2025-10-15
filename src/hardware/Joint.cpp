#include "Joint.hpp"

Joint::Joint(Motor &motor, AS5600Sensor &sensor, CurrentSensor &current) :
motor_(motor), sensor_(sensor), current_(current) {

}

void Joint::initialize() {
    motor_.initialize();
    sensor_.initialize();
    current_.initialize();
}

//-------------flag
float Joint::getAngle() {
    sensor_.readAngle();
    return sensor_.getDegrees();
}

float Joint::getCurrent() {
    return current_.readCurrent();
}

void Joint::drive(float controlSignal, int pwmDuty) {
    motor_.driveMotor(controlSignal, pwmDuty);
}

void Joint::stop() {
    motor_.stopMotor();
}

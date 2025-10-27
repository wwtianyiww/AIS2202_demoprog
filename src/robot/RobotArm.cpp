#include "RobotArm.hpp"

RobotArm::RobotArm(const JointController& shoulderController,
                   const JointController& wristController,
                   const StateMonitor& monitor)
    : shoulderController_(shoulderController), 
      wristController_(wristController),
      monitor_(monitor) {
}

void RobotArm::initialize() {
    monitor_.initialize();
    monitor_.logError("Robot Arm initializing...");
    
    // Joint controllers are already initialized through their hardware components
    // Just verify they're ready
    monitor_.logJointState("Shoulder Init", 
        shoulderController_.getAngle(),
        shoulderController_.getError(),
        shoulderController_.getCurrent());
    
    monitor_.logJointState("Wrist Init", 
        wristController_.getAngle(),
        wristController_.getError(),
        wristController_.getCurrent());
    
    Serial.println("Robot Arm initialized successfully\n");
}

void RobotArm::update(float dt) {
    // Update both joint controllers
    shoulderController_.update(dt);
    wristController_.update(dt);
}

void RobotArm::setShoulderTarget(float angle) {
    shoulderController_.setTarget(angle);
    
    Serial.print("Shoulder target set to: ");
    Serial.print(angle, 1);
    Serial.println("°");
}

void RobotArm::setWristTarget(float angle) {
    wristController_.setTarget(angle);
    
    Serial.print("Wrist target set to: ");
    Serial.print(angle, 1);
    Serial.println("°");
}

void RobotArm::setTargets(float shoulder, float wrist) {
    shoulderController_.setTarget(shoulder);
    wristController_.setTarget(wrist);
    
    Serial.print("Targets set - Shoulder: ");
    Serial.print(shoulder, 1);
    Serial.print("°, Wrist: ");
    Serial.print(wrist, 1);
    Serial.println("°");
}

void RobotArm::homeAll() {
    setTargets(0.0f, 0.0f);
    Serial.println("Homing both joints to 0°");
}

float RobotArm::getShoulderAngle() {
    return shoulderController_.getAngle();
}

float RobotArm::getWristAngle() {
    return wristController_.getAngle();
}

float RobotArm::getShoulderError() {
    return shoulderController_.getError();
}

float RobotArm::getWristError() {
    return wristController_.getError();
}

float RobotArm::getShoulderCurrent() {
    return shoulderController_.getCurrent();
}

float RobotArm::getWristCurrent() {
    return wristController_.getCurrent();
}

uint16_t RobotArm::getRawShoulder() {
    return shoulderController_.getRawAngle();
}

uint16_t RobotArm::getRawWrist() {
    return wristController_.getRawAngle();
}


void RobotArm::printStatus() {
    monitor_.logBothJoints(
        getShoulderAngle(),
        getRawShoulder(),
        getShoulderError(),
        getShoulderCurrent(),
        getWristAngle(),
        getRawWrist(),
        getWristError(),
        getWristCurrent()
    );
}
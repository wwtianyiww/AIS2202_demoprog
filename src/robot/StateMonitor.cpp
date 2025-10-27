#include "StateMonitor.hpp"


StateMonitor::StateMonitor(long baudRate)
    : baudRate_(baudRate), verbose_(false) {
}

void StateMonitor::initialize() {
    Serial.begin(baudRate_);

    // Wait for serial to initialize
    delay(1000);

    Serial.println("\n=== State Monitor Initialized ===");
    Serial.print("Baud Rate: ");
    Serial.println(baudRate_);
    Serial.println("=================================\n");
}

void StateMonitor::logJointState(const char* name, float angle, float error, float current) {
    Serial.print(name);
    Serial.print(": ");
    Serial.print(angle, 2);
    Serial.print("° (err: ");
    Serial.print(error, 2);
    Serial.print("°, I: ");
    Serial.print(current, 1);
    Serial.println(" mA)");
}

void StateMonitor::logBothJoints(float shoulderAngle, uint16_t rawShoulder, float shoulderError, float shoulderCurrent,
                                 float wristAngle,uint16_t rawWrist , float wristError, float wristCurrent) {
    if (verbose_) {
        Serial.println("--- Joint Status ---");
    }

    Serial.print("Shoulder: ");
    Serial.print(shoulderAngle, 2);
    Serial.print("° rawangle: ");
    Serial.print(rawShoulder);
    Serial.print(" (err: ");
    Serial.print(shoulderError, 2);
    Serial.print("°, I: ");
    Serial.print(shoulderCurrent, 1);
    Serial.print(" mA) | ");

    Serial.print("Wrist: ");
    Serial.print(wristAngle, 2);
    Serial.print("° rawangle: ");
    Serial.print(rawWrist);
    Serial.print(" (err: ");
    Serial.print(wristError, 2);
    Serial.print("°, I: ");
    Serial.print(wristCurrent, 1);
    Serial.println(" mA)");

    if (verbose_) {
        Serial.println("--------------------\n");
    }
}

void StateMonitor::logError(const char* message) {
    Serial.print("[ERROR] ");
    Serial.println(message);
}

void StateMonitor::setVerbose(bool verbose) {
    verbose_ = verbose;

    if (verbose_) {
        Serial.println("Verbose mode: ON");
    } else {
        Serial.println("Verbose mode: OFF");
    }
}
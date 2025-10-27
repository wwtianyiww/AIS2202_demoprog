#include <Arduino.h>
#include <Wire.h>

// Hardware components
#include "hardware/Motor.hpp"
#include "hardware/AS5600Sensor.hpp"
#include "hardware/CurrentSensor.hpp"
#include "hardware/Joint.hpp"

// Control components
#include "control/PIDController.hpp"
#include "control/DeadbandController.hpp"
#include "control/JointController.hpp"

// Utilities
#include "util/I2CBusManager.hpp"
#include "util/AngleMath.hpp"

// Configuration
#include "config/HardwareConfig.hpp"
#include "config/ControlConfig.hpp"
#include "config/Constants.hpp"

//Robot
#include "robot/RobotArm.hpp"
#include "robot/StateMonitor.hpp"

using namespace HardwareConfig;
using namespace ControlConfig;
using namespace Constants;

void printStatus();
void handleSerialCommand();

bool motorsEnabled = true;

// ==================== Shoulder Joint ====================
Motor shoulderMotor(shoulderINA1, shoulderINA2, shoulderPWMA);
AS5600Sensor shoulderAngleSensor(Wire, 0.0f, 45.0f);
CurrentSensor shoulderCurrentSensor(shoulderCurrentSensorPin, currentSensitivity);
Joint shoulderJoint(shoulderMotor, shoulderAngleSensor, shoulderCurrentSensor);
PIDController shoulderPID(shoulderKp, shoulderKi, shoulderKd, shoulderMaxIntegral);
DeadbandController shoulderDeadband(shoulderPID, shoulderDeadbandThreshold);
JointController shoulderController(shoulderJoint, shoulderDeadband);

// ==================== Wrist Joint ====================
Motor wristMotor(wristINA1, wristINA2, wristPWMA);
AS5600Sensor wristAngleSensor(Wire1, 4.0f);
CurrentSensor wristCurrentSensor(wristCurrentSensorPin, currentSensitivity);
Joint wristJoint(wristMotor, wristAngleSensor, wristCurrentSensor);
PIDController wristPID(wristKp, wristKi, wristKd, wristMaxIntegral);
DeadbandController wristDeadband(wristPID, wristDeadbandThreshold);
JointController wristController(wristJoint, wristDeadband);

// ==================== State Monitor ====================
StateMonitor monitor(9600);

// ==================== Robot Arm ====================
RobotArm robot(shoulderController, wristController, monitor);

// ==================== Timing ====================
uint32_t lastUpdateTime = 0;
uint32_t lastPrintTime = 0;

void setup() {
    // Initialize I2C buses
    I2CBusManager::initializeBus(Wire, shoulderMotorSDAPin, shoulderMotorSCLPin, I2CFrequency, I2CTimeout);
    I2CBusManager::initializeBus(Wire1, wristMotorSDAPin, wristMotorSCLPin, I2CFrequency, I2CTimeout);

    // Initialize joints
    shoulderJoint.initialize();
    wristJoint.initialize();

    // Initialize robot
    robot.initialize();

    lastUpdateTime = millis();
    lastPrintTime = millis();

	// ADD THIS AT THE END OF setup()
    Serial.println("\n=== DIRECT I2C TEST IN MAIN ===");
    delay(100);

    // Test reading directly
    uint16_t rawShoulder = 0, rawWrist = 0;

    Wire.beginTransmission(0x36);
    Wire.write(0x0E);
    if (Wire.endTransmission() == 0 && Wire.requestFrom((int)0x36, 2) == 2) {
        uint16_t hi = Wire.read();
        uint16_t lo = Wire.read();
        rawShoulder = ((hi << 8) | lo) & 0x0FFF;
        Serial.printf("Direct Shoulder Read: %u (%.2f°)\n",
                      rawShoulder, rawShoulder * 360.0f / 4096.0f);
    }

    Wire1.beginTransmission(0x36);
    Wire1.write(0x0E);
    if (Wire1.endTransmission() == 0 && Wire1.requestFrom((int)0x36, 2) == 2) {
        uint16_t hi = Wire1.read();
        uint16_t lo = Wire1.read();
        rawWrist = ((hi << 8) | lo) & 0x0FFF;
        Serial.printf("Direct Wrist Read: %u (%.2f°)\n",
                      rawWrist, rawWrist * 360.0f / 4096.0f);
    }

    Serial.println("==================================\n");



}

void loop() {
    uint32_t currentTime = millis();

    // Control loop (100Hz) - only run if motors enabled
    if (motorsEnabled && currentTime - lastUpdateTime >= controlLoopInterval) {
        float dt = (currentTime - lastUpdateTime) / 1000.0f;
        lastUpdateTime = currentTime;

        robot.update(dt);
    }

    // Status printing (1Hz)
    if (currentTime - lastPrintTime >= 1000) {
        lastPrintTime = currentTime;
        robot.printStatus();
    }

    // Handle serial commands
    if (Serial.available()) {
        handleSerialCommand();
    }
}

void handleSerialCommand() {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) return;

    char command = cmd.charAt(0);

    switch (command) {
        case 's':
        case 'S':
            if (cmd.length() > 1) {
                robot.setShoulderTarget(cmd.substring(1).toFloat());
            }
            break;
        case 'w':
        case 'W':
            if (cmd.length() > 1) {
                robot.setWristTarget(cmd.substring(1).toFloat());
            }
            break;
        case 'h':
        case 'H':
            robot.homeAll();
            break;
        case 'p':
        case 'P':
            robot.printStatus();
            break;
        case 'm':  // NEW: Motor toggle
        case 'M':
            motorsEnabled = !motorsEnabled;
            if (!motorsEnabled) {
                shoulderJoint.stop();
                wristJoint.stop();
                Serial.println("\n*** MOTORS DISABLED ***");
                Serial.println("You can now move joints manually by hand");
                Serial.println("Send 'm' again to re-enable motors\n");
            } else {
                Serial.println("\n*** MOTORS ENABLED ***\n");
            }
            break;
        case 'c':
        case 'C':
            Serial.print("Calibrating for zero position");
            shoulderAngleSensor.calibrateZero();
            wristAngleSensor.calibrateZero();
            Serial.println("Calibration complete! Current position is now 0°");
            break;
        default:
            Serial.println("Unknown command");
            break;
    }
}
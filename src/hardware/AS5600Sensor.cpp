#include "AS5600Sensor.hpp"

AS5600Sensor::AS5600Sensor(TwoWire& wire, float gearRatio)
    : wire_(wire), sensor_(nullptr), lastRaw_(0), encoderDegrees_(0.0f), gearRatio_(gearRatio), degrees_(0.0f) {
}

AS5600Sensor::~AS5600Sensor() {
    if (sensor_ != nullptr) {
        delete sensor_;
    }
}

bool AS5600Sensor::initialize() {
    // Construct the AS5600 object
    sensor_ = new AS5600(&wire_);

    Serial.println("[DEBUG] AS5600 object created");

    // Call begin()
    sensor_->begin();
    Serial.println("[DEBUG] sensor_->begin() called");

    if (!sensor_->isConnected()) {
        Serial.println("[DEBUG] sensor_ NOT connected!");
        return false;
    }
    Serial.println("[DEBUG] sensor_ is connected");

    int magStatus = getMagnetStatus();
    Serial.printf("[DEBUG] Magnet status: %d\n", magStatus);

    return readAngle();
}
bool AS5600Sensor::readAngle() {
    if (sensor_ == nullptr) {
        return false;
    }

    int rawFromLibrary = sensor_->readAngle();

    if (rawFromLibrary < 0) {
        return false;
    }

    lastRaw_ = static_cast<uint16_t>(rawFromLibrary);
    encoderDegrees_ = AngleMath::rawToDegrees(lastRaw_);
    degrees_ = encoderDegrees_/ gearRatio_;
    return true;  // Clean and fast!
}

uint16_t AS5600Sensor::getRawValue() const {
    return lastRaw_;
}

float AS5600Sensor::getDegrees() const {
    return degrees_;
}

bool AS5600Sensor::isConnected() {
    if (sensor_ == nullptr) {
        return false;
    }
    return sensor_->isConnected();
}

int AS5600Sensor::getMagnetStatus() {
    if (sensor_ == nullptr) {
        return -1;
    }

    if (sensor_->magnetTooStrong()) return 1;
    if (sensor_->magnetTooWeak()) return 2;
    if (!sensor_->detectMagnet()) return 3;
    return 0;
}

bool AS5600Sensor::setZeroPosition(uint16_t position) {
    if (sensor_ == nullptr) {
        return false;
    }
    sensor_->setZPosition(position);
    return true;
}

uint16_t AS5600Sensor::getZeroPosition() {
    if (sensor_ == nullptr) {
        return 0;
    }
    return sensor_->getZPosition();
}

bool AS5600Sensor::setMaxPosition(uint16_t position) {
    if (sensor_ == nullptr) {
        return false;
    }
    sensor_->setMPosition(position);
    return true;
}

uint16_t AS5600Sensor::getMaxPosition() {
    if (sensor_ == nullptr) {
        return 0;
    }
    return sensor_->getMPosition();
}

bool AS5600Sensor::calibrateZero() {
    if (sensor_ == nullptr) {
        return false;
    }

    // Read current position
    if (!readAngle()) {
        Serial.println("[CALIBRATE] Failed to read angle");
        return false;
    }

    uint16_t currentRaw = getRawValue();

    // Set this as new zero position
    sensor_->setZPosition(currentRaw);

    Serial.printf("[CALIBRATE] Set zero position to raw: %d (%.2f encoder degrees)\n",
                  currentRaw, encoderDegrees_);

    // Verify
    uint16_t storedZero = sensor_->getZPosition();
    Serial.printf("[CALIBRATE] Verified stored zero: %d\n", storedZero);

    return (storedZero == currentRaw);
}
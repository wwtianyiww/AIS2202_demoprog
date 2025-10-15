#ifndef DEMOPROG1_AS5600SENSOR_HPP
#define DEMOPROG1_AS5600SENSOR_HPP
#include <AS5600.h>
#include <util/AngleMath.hpp>

class AS5600Sensor {
public:
    AS5600Sensor(TwoWire& wire, float gearRatio = 1.0f);
    ~AS5600Sensor();  // Add destructor
    bool initialize();
    bool readAngle();
    uint16_t getRawValue() const;
    float getDegrees() const;
    bool isConnected();
    int getMagnetStatus();

    // Offset/homing methods for using library features
    bool setZeroPosition(uint16_t position);
    uint16_t getZeroPosition();
    bool setMaxPosition(uint16_t position);
    uint16_t getMaxPosition();
    bool calibrateZero();

private:
    TwoWire& wire_;
    AS5600* sensor_;  // Changed to pointer!
    uint16_t lastRaw_;
    float encoderDegrees_;
    float gearRatio_;
    float degrees_;
};
#endif //DEMOPROG1_AS5600SENSOR_HPP
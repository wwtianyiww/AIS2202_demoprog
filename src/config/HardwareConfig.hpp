#ifndef DEMOPROG1_HARDWARECONFIG_HPP
#define DEMOPROG1_HARDWARECONFIG_HPP
#include <cstdint>
namespace HardwareConfig {
    //---------------- Pins ----------------
    constexpr int shoulderINA1 = 19;
    constexpr int shoulderINA2 = 18;
    constexpr int shoulderPWMA = 5;

    constexpr int wristINA1 = 25;
    constexpr int wristINA2 = 26;
    constexpr int wristPWMA = 27;



    // AS5600
    static constexpr uint8_t HFSensorAdr = 0x36;

    // I2C pins
    constexpr int shoulderMotorSDAPin = 21;
    constexpr int shoulderMotorSCLPin = 22;

    constexpr int wristMotorSDAPin = 16;
    constexpr int wristMotorSCLPin = 17;

    // Current sensors
    constexpr int shoulderCurrentSensorPin = 34;
    constexpr int wristCurrentSensorPin    = 32;
}
#endif //DEMOPROG1_HARDWARECONFIG_HPP
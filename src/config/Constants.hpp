#ifndef DEMOPROG1_CONSTANTS_HPP
#define DEMOPROG1_CONSTANTS_HPP
#include <cstdint>
namespace Constants {
    // 400 mV/A -> 0.4 mV/mA -> 2.5 mA per mV
    float currentSensitivity = 2.5f; //_mA_per_mV
    float currentDividerGain = 1.0f;      // set >1.0 if you added an ADC divider
    constexpr int AVG_SAMPLES = 10;
    constexpr uint32_t  I2CFrequency = 400000; //fast mode from AS5600 library
    constexpr uint16_t  I2CTimeout = 50; //from AS5600 library
    constexpr uint32_t controlLoopInterval = 10;
}

#endif //DEMOPROG1_CONSTANTS_HPP
#ifndef DEMOPROG1_I2CBUSMANAGER_HPP
#define DEMOPROG1_I2CBUSMANAGER_HPP
#include <Wire.h>

class I2CBusManager {
public:
    static void initializeBus(TwoWire& wire, int sda, int scl,
                             uint32_t frequency = 100000,
                             uint16_t timeout = 50);
    static bool testBus(TwoWire& wire, uint8_t address);
    I2CBusManager() = delete;
private:

};
#endif //DEMOPROG1_I2CBUSMANAGER_HPP
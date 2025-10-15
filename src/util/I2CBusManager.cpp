#include "I2CBusManager.hpp"

void I2CBusManager::initializeBus(TwoWire& wire, int sda, int scl,
                                  uint32_t frequency, uint16_t timeout) {
    // Initialize I2C bus with custom SDA and SCL pins
    wire.begin(sda, scl);

    // Set I2C clock frequency (default 100kHz, can go up to 400kHz for fast mode)
    wire.setClock(frequency);

    // Set timeout in milliseconds to prevent hanging on bus errors
    wire.setTimeOut(timeout);
}

bool I2CBusManager::testBus(TwoWire& wire, uint8_t address) {
    // Begin transmission to the specified I2C address
    wire.beginTransmission(address);

    // End transmission and check for ACK
    // Returns 0 if device acknowledged, non-zero for errors:
    // 1: data too long to fit in transmit buffer
    // 2: received NACK on transmit of address
    // 3: received NACK on transmit of data
    // 4: other error
    // 5: timeout
    uint8_t error = wire.endTransmission();

    // Return true if device responded (error == 0)
    return (error == 0);
}
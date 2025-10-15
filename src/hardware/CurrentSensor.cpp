#include "CurrentSensor.hpp"
#include "CurrentSensor.hpp"

CurrentSensor::CurrentSensor(int sensorPin, float sensitivity_mA_per_mV) :
sensorPin_(sensorPin), vRef_(0.0f), sensitivity_(sensitivity_mA_per_mV) {
}

void CurrentSensor::calibrate() {
    // Average multiple readings to get the zero-current reference voltage
    long sum = 0;
    for (int i = 0; i < AVGSAMPLES; i++) {
        sum += analogRead(sensorPin_);
        delay(10); // Small delay between samples
    }

    // Convert average ADC value to millivolts
    // Assuming 10-bit ADC (0-1023) and 3.3V or 5V reference
    // For 3.3V: (sum / AVGSAMPLES) * (3300.0f / 1023.0f)
    // For 5V: (sum / AVGSAMPLES) * (5000.0f / 1023.0f)
    float avgADC = sum / (float)AVGSAMPLES;
    vRef_ = avgADC * (3300.0f / 1023.0f); // Adjust 3300.0f to 5000.0f if using 5V
}

void CurrentSensor::initialize() {
    pinMode(sensorPin_, INPUT);
    calibrate(); // Calibrate on initialization to set zero reference
}


float CurrentSensor::readCurrent() {
    // Average multiple samples for stability
    long sum = 0;
    for (int i = 0; i < AVGSAMPLES; i++) {
        sum += analogRead(sensorPin_);
    }

    // Convert to millivolts
    float avgADC = sum / (float)AVGSAMPLES;
    float voltage_mV = avgADC * (3300.0f / 1023.0f); // Adjust if using 5V

    // Calculate voltage difference from reference (zero current point)
    float deltaV_mV = voltage_mV - vRef_;


    // For ACS712: sensitivity is typically around 185mV/A = 0.185mV/mA = 5.41mA/mV
    float current_mA = deltaV_mV * sensitivity_;

    return current_mA;
}
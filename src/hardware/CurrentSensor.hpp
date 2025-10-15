#ifndef DEMOPROG1_CURRENTSENSOR_HPP
#define DEMOPROG1_CURRENTSENSOR_HPP
#include "Arduino.h"
class CurrentSensor {
public:
    explicit CurrentSensor(int sensorPin, float sensitivity_mA_per_mV = 2.5f);
    void initialize();
    void calibrate();
    float readCurrent();
private:
    int sensorPin_;
    float vRef_; //mV
    float sensitivity_;
    static const int AVGSAMPLES = 10;
};

#endif //DEMOPROG1_CURRENTSENSOR_HPP
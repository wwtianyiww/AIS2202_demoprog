#include <Arduino.h>
#include <Wire.h>
#include <array>

//shoulder control pins
const int shoulderINA1 = 19;
const int shoulderINA2 = 18;
const int shoulderPWMA = 5;

//wrist control pins
const int wristINA1 = 25;
const int wristINA2 = 26;
const int wristPWMA = 27;

//"AS5600L I2C Address is programmable" from DS
//"The slave address is 0x40" from DS
// Hall effect sensor - wrist motor
const int shoulderMotorSDAPin = 21;
const int shoulderMotorSCLPin = 22;
uint8_t HFSensorAdr = 0x40; //0x40 eller 0x365

// Hall effect sensor - wrist motor
const int wristMotorSDAPin = 16;
const int wristMotorSCLPin = 17;


// --- NY: Strømsensor pin ---
const int shoulderCurrentSensorPin = 34;
const int wristCurrentSensorPin = 32;


// --- NY: Kalibreringsverdier for strømsensor ---
float currentSensitivity = 1; // 100mA per 500mV = 0.2 (juster etter kalibrering)
//float currentVref = 2500; // 2.5V i mV (juster etter kalibrering)
const int AVG_SAMPLES = 10; // Antall samples for glatting


struct pidState {
  float integ = 0.0;
  float prevError = 0.0;
};

pidState pidShoulder, pidWrist;

uint32_t prevMS = 0;
float targetAngleShoulder = 90.0;
float targetAngleWrist = 90.0;
float currentVrefShoulder = 2500.0f;  // will be overwritten in setup()
float currentVrefWrist    = 2500.0f;  // will be overwritten in setup()

/*
// --- Eksisterende hjelpefunksjoner ---
bool i2cScanFindAS560x(uint8_t &foundAdr) {
  Serial.println("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  - 0x%02X\n", a);
      if (a == 0x40 || a == 0x36) {
        foundAdr = a;   // update caller's variable
        return true;    // stop once we find one
      }
    }
  }
  return false; // nothing found
}
*/
float measureZeroMv(int pin) {
  long sum=0; for(int i=0;i<100;i++){ sum+=analogReadMilliVolts(pin); delay(2); }
  return sum/100.0f;
}


bool readAngleShoulder(uint16_t &raw) {
  Wire.beginTransmission(HFSensorAdr);
  Wire.write(0x0E);
  if (Wire.endTransmission(true) != 0) return false;
  int n = Wire.requestFrom((int)HFSensorAdr, 2);
  if (n != 2) return false;
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  raw = ((hi << 8) | lo) & 0x0FFF;
  return true;
}

bool readAngleWrist(uint16_t &raw) {
  Wire1.beginTransmission(HFSensorAdr);
  Wire1.write(0x0E);
  if (Wire1.endTransmission(true) != 0) return false;
  int n = Wire1.requestFrom((int)HFSensorAdr, 2);
  if (n != 2) return false;
  uint16_t hi = Wire1.read();
  uint16_t lo = Wire1.read();
  raw = ((hi << 8) | lo) & 0x0FFF;
  return true;
}



/*
float readAngle(uint16_t &raw, std::uint8_t adr) {
  if (!adr) return -999.000; //errorcode -999.000 no adr
  Wire.beginTransmission(adr);
  Wire.write(0x0E);
  if (Wire.endTransmission(true) != 0) return -999.001; //error code -999.001 end transmission
  int n = Wire.requestFrom((int)adr, 2);
  if (n != 2) return -999.002; //errorcode -999.002 did not recieve both bytes
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  raw = ((hi << 8) | lo) & 0x0FFF;
  float deg = raw * (360.0f / 4096.0f);
  return deg;
}
*/

// --- NY: Funksjon for å lese strøm ---
float readCurrent(int currentSensor, float vRef) {
  float sensorValue = 0;

  // Gjennomsnitt av flere målinger
  for (int i = 0; i < AVG_SAMPLES; i++) {
    //sensorValue += analogRead(currentSensor);
    sensorValue += analogReadMilliVolts(currentSensor);
    delay(2);
  }

  sensorValue = sensorValue / AVG_SAMPLES;
  float voltage = sensorValue; //* (5000.0 / 4095.0); // ESP32 ADC: 0-4095 -> 0-5000mV
  float current = (voltage - vRef) * currentSensitivity;

  return current;
}


float error(const float target, float currentAngle) {
  float e = currentAngle - target;
  return e;
}


float kP = 0.4, kI = 0.2, kD = 0.2;
float maxI = 200.0;

float pidStep(float error, float dt,pidState& state, float kP, float kI, float kD) {
  state.integ += error * dt;


  float deriv = (error - state.prevError)/dt;
  state.prevError = error;

  float u = kP * error + kI * state.integ + kD * deriv;
  return u;
}

void driveMotor(float pid, const int pin1, const int pin2, const int pinPWM) {
  if (pid >= 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pinPWM, 50);
  }else {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pinPWM, 50);
  }
}





void setup() {
  delay(300);
  Serial.begin(9600);

  //Shoulder
  pinMode(shoulderINA1, OUTPUT);
  pinMode(shoulderINA2, OUTPUT);
  pinMode(shoulderPWMA, OUTPUT);
  //wrist
  pinMode(wristINA1, OUTPUT);
  pinMode(wristINA2, OUTPUT);
  pinMode(wristPWMA, OUTPUT);
  /*
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  digitalWrite(PWMA, HIGH);
*/

  // --- Eksisterende I2C-oppsett ---
  Wire.begin(shoulderMotorSDAPin, shoulderMotorSCLPin);
  Wire1.begin(wristMotorSDAPin, wristMotorSCLPin);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  //i2cScanFindAS560x(shoulderHFSensorAdr);
  //i2cScanFindAS560x(wristHFSensorAdr);

  // --- NY: Strømsensor oppsett ---
  pinMode(shoulderCurrentSensorPin, INPUT);
  pinMode(wristCurrentSensorPin, INPUT);
  analogReadResolution(12); // Bruk 12-bit oppløsning på ESP32
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(shoulderCurrentSensorPin, ADC_11db);
  analogSetPinAttenuation(wristCurrentSensorPin,    ADC_11db);
  float currentVrefShoulder = measureZeroMv(shoulderCurrentSensorPin);
  float currentVrefWrist = measureZeroMv(wristCurrentSensorPin);

  prevMS = millis();

  Serial.println("Motor + AS5600(L) + Strømsensor test");



}

void loop() {
  // --- Eksisterende kode for vinkelmåling ---
  uint16_t rawShoulder;
  uint16_t rawWrist;

  //time
  uint32_t now = millis();
  float dt = (now - prevMS) / 1000.0f;
  if (dt <= 0) dt = 1e-3f;
  prevMS = now;

  float degShoulder = readAngleShoulder(rawShoulder);
  float degWrist = readAngleWrist(rawWrist);
  float currentShoulder = readCurrent(shoulderCurrentSensorPin, currentVrefShoulder);
  float currentWrist = readCurrent(wristCurrentSensorPin, currentVrefWrist);
  Serial.printf("[Shoulder]:  raw=%u  deg=%.2f strøm=%.2fmA\n / [Wrist]:  raw=%u  deg=%.2f strøm=%.2fmA\n", rawShoulder, degShoulder, currentShoulder,rawWrist, degWrist, currentWrist);


  float eShoulder = error(targetAngleShoulder, degShoulder);
  float eWrist = error(targetAngleWrist, degWrist);

  float uShoulder = pidStep(eShoulder, dt, pidShoulder,kP, kI, kD);
  float uWrist = pidStep(eWrist, dt, pidWrist,kP, kI, kD);


  driveMotor(uShoulder, shoulderINA1, shoulderINA2, shoulderPWMA);
  driveMotor(uWrist, wristINA1, wristINA2, wristPWMA);


  delay(5);
}
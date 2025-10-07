#include <Arduino.h>
#include <Wire.h>

//---------------- Pins ----------------
const int shoulderINA1 = 19;
const int shoulderINA2 = 18;
const int shoulderPWMA = 5;

const int wristINA1 = 25;
const int wristINA2 = 26;
const int wristPWMA = 27;

// AS5600 standard address (NOT L)
static constexpr uint8_t HFSensorAdr = 0x36;

// I2C pins
const int shoulderMotorSDAPin = 21;
const int shoulderMotorSCLPin = 22;

const int wristMotorSDAPin = 16;
const int wristMotorSCLPin = 17;

// Current sensors
const int shoulderCurrentSensorPin = 34;
const int wristCurrentSensorPin    = 32;

//---------------- Config ----------------
float thresholdAngle = 1.0f;
float thresholdAngleWrist = 5.0f; // stop band (deg)
uint32_t prevMS = 0;
float targetAngleShoulder = 200.0f;
float targetAngleWrist    =  190.0f;

// 400 mV/A -> 0.4 mV/mA -> 2.5 mA per mV
float currentSensitivity_mA_per_mV = 2.5f;
float currentDividerGain = 1.0f;      // set >1.0 if you added an ADC divider
const int AVG_SAMPLES = 10;

//---------------- PID ----------------
struct pidState { float integ = 0.0f; float prevError = 0.0f; };
pidState pidShoulder, pidWrist;
float kP = 0.4f, kI = 0.2f, kD = 0.2f;
float maxI = 200.0f;

//---------------- Globals (DO NOT shadow) ----------------
float currentVrefShoulder = 2500.0f;  // measured in setup()
float currentVrefWrist    = 2500.0f;  // measured in setup()
float degShoulder = 0.0f;
float degWrist    = 0.0f;

//---------------- Helpers ----------------
float measureZeroMv(int pin) {
  long sum = 0;
  for (int i=0; i<100; ++i) { sum += analogReadMilliVolts(pin); delay(2); }
  return (sum / 100.0f) * currentDividerGain;
}

bool readAngleOnBus(TwoWire& bus, uint16_t &raw) {
  bus.beginTransmission(HFSensorAdr);
  bus.write(0x0E);                        // RAW_ANGLE high byte
  if (bus.endTransmission(true) != 0) return false;
  int n = bus.requestFrom((int)HFSensorAdr, 2);
  if (n != 2) return false;
  uint16_t hi = bus.read();
  uint16_t lo = bus.read();
  raw = ((hi << 8) | lo) & 0x0FFF;       // 12-bit
  return true;
}

bool readAngleShoulder(uint16_t &raw) { return readAngleOnBus(Wire,  raw); }
bool readAngleWrist   (uint16_t &raw) { return readAngleOnBus(Wire1, raw); }

inline float rawToDeg(uint16_t raw) { return raw * (360.0f / 4096.0f); }

// shortest-path wrapped error [-180,180]
float angleError(float target, float measured) {
  float e = target - measured;
  while (e >  180.0f) e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return e;
}

float readCurrent_mA(int pin, float vRef_mV) {
  float mv = 0.0f;
  for (int i=0; i<AVG_SAMPLES; ++i) { mv += analogReadMilliVolts(pin); delay(2); }
  mv = (mv / AVG_SAMPLES) * currentDividerGain;
  return (mv - vRef_mV) * currentSensitivity_mA_per_mV;
}

float pidStep(float err, float dt, pidState& s, float kP, float kI, float kD) {
  s.integ += err * dt;
  if (s.integ >  maxI) s.integ =  maxI;
  if (s.integ < -maxI) s.integ = -maxI;
  float deriv = (dt > 1e-6f) ? (err - s.prevError)/dt : 0.0f;
  s.prevError = err;
  return kP*err + kI*s.integ + kD*deriv;
}

void stopMotor(int in1, int in2, int pwm) {
  analogWrite(pwm, 0);         // coast
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void driveMotor(float pid, int pin1, int pin2, int pinPWM) {
  const int duty = 50;         // fixed duty (simple). Change to proportional if desired.
  if (pid >= 0.0f) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pinPWM, duty);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pinPWM, duty);
  }
}

//---------------- Setup ----------------
void setup() {
  delay(300);
  Serial.begin(9600);        // match your monitor

  pinMode(shoulderINA1, OUTPUT);
  pinMode(shoulderINA2, OUTPUT);
  pinMode(shoulderPWMA, OUTPUT);
  pinMode(wristINA1, OUTPUT);
  pinMode(wristINA2, OUTPUT);
  pinMode(wristPWMA, OUTPUT);

  Wire.begin (shoulderMotorSDAPin, shoulderMotorSCLPin);
  Wire1.begin(wristMotorSDAPin,    wristMotorSCLPin);
  Wire.setClock(100000);   Wire.setTimeOut(50);
  Wire1.setClock(100000);  Wire1.setTimeOut(50);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(shoulderCurrentSensorPin, ADC_11db);
  analogSetPinAttenuation(wristCurrentSensorPin,    ADC_11db);

  // IMPORTANT: assign to globals (no 'float' here)
  currentVrefShoulder = measureZeroMv(shoulderCurrentSensorPin);
  currentVrefWrist    = measureZeroMv(wristCurrentSensorPin);

  prevMS = millis();
  Serial.println("Start: angles + current + PID + deadband");
}

//---------------- Loop ----------------
void loop() {
  // time
  uint32_t now = millis();
  float dt = (now - prevMS) / 1000.0f;
  if (dt <= 0.0f) dt = 1e-3f;
  prevMS = now;

  // 1) READ ANGLES FIRST (update GLOBALS; DO NOT redeclare 'float')
  uint16_t rawWrist = 0, rawShoulder = 0;

  if (readAngleWrist(rawWrist)) {
    degWrist = rawToDeg(rawWrist);
  } else {
    Serial.print("lesefeil(wrist) ");
  }

  if (readAngleShoulder(rawShoulder)) {
    degShoulder = rawToDeg(rawShoulder);
  } else {
    Serial.print("lesefeil(shoulder) ");
  }

  // 2) COMPUTE ERRORS with updated angles
  float eWrist    = angleError(targetAngleWrist,    degWrist);
  float eShoulder = angleError(targetAngleShoulder, degShoulder);

  // 3) PID
  float uWrist    = pidStep(eWrist,    dt, pidWrist,    kP, kI, kD);
  float uShoulder = pidStep(eShoulder, dt, pidShoulder, kP, kI, kD);

  // 4) DEAD-BAND STOP: drive only when |error| > thresholdAngle
  if (fabsf(eWrist) > thresholdAngleWrist)    driveMotor(uWrist,    wristINA1,    wristINA2,    wristPWMA);
  else                                   stopMotor (wristINA1, wristINA2,    wristPWMA);

  if (fabsf(eShoulder) > thresholdAngle) driveMotor(uShoulder, shoulderINA1, shoulderINA2, shoulderPWMA);
  else                                   stopMotor (shoulderINA1, shoulderINA2, shoulderPWMA);

  // 5) Currents (mA) and print
  float currentShoulder = readCurrent_mA(shoulderCurrentSensorPin, currentVrefShoulder);
  float currentWrist    = readCurrent_mA(wristCurrentSensorPin,    currentVrefWrist);

  Serial.printf("[Wrist] raw=%4u deg=%7.2f err=%+7.2f  |  "
                "[Shoulder] raw=%4u deg=%7.2f err=%+7.2f  |  "
                "I_sh=%+7.1fmA I_wr=%+7.1fmA\n",
                (unsigned)rawWrist,   degWrist,    eWrist,
                (unsigned)rawShoulder,degShoulder, eShoulder,
                currentShoulder, currentWrist);

  delay(5);
}

#include "Sensors.h"

Adafruit_BME280 bme;
Adafruit_SHT4x sht4x;

// Forward declaration of SafeMode handler (to be defined in main or separate file)
extern void enterSafeMode(const char* reason);

void initializeSensors() {
  bool bme_ok = bme.begin(0x76);
  bool sht_ok = sht4x.begin();

  if (!bme_ok || !sht_ok) {
    if (!bme_ok) Serial.println("BME280 init failed!");
    if (!sht_ok) Serial.println("SHT4x init failed!");
    enterSafeMode("I2C Init Failure");
    return;
  }

  sht4x.setPrecision(SHT4X_HIGH_PRECISION);
  sht4x.setHeater(SHT4X_NO_HEATER);

  pinMode(coolerPin, OUTPUT);
  analogWrite(coolerPin, 0);
  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW);
}

bool checkSensorHealth() {
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  if (t < -40.0f || t > 85.0f || h < 0.0f || h > 100.0f || std::isnan(t) || std::isnan(h)) return false;
  return true;
}

void performSelfTest() {
  Serial.println("--- System Self-Test ---");
  bool pass = checkSensorHealth();
  float p = bme.readPressure() / 100.0f;
  if (p < 800.0f || p > 1200.0f) pass = false;
  if (!pass) enterSafeMode("Self-Test Failed");
  else Serial.println("Self-Test: SUCCESS");
}

#include "ThermalControl.h"
#include "Sensors.h"
#include "Physics.h"

extern float coolingHealth;

void stopCooling() {
  analogWrite(coolerPin, 0);
}

void controlCoolingPWM(float targetTemperature, float ambientRefTemp) {
  float integral = 0.0f;
  unsigned long startMs = millis();
  unsigned long lastMs = startMs;
  float currentTemp = bme.readTemperature();
  float lastError = currentTemp - targetTemperature;

  while (true) {
    unsigned long now = millis();
    if (now - startMs > coolingTimeoutMs) break;

    float dtReal = (float)(now - lastMs) / 1000.0f;
    if (dtReal < 0.001f) dtReal = 0.001f;
    lastMs = now;

    currentTemp = bme.readTemperature();
    float error = currentTemp - targetTemperature;
    if (fabsf(error) < 0.1f) break;

    float P = Kp * error;
    if (fabsf(error) < 2.0f) integral += error * dtReal;
    else integral = 0.0f;
    float I = Ki * integral;
    float D = Kd * (error - lastError) / dtReal;
    lastError = error;
    float FF = Kf * (ambientRefTemp - targetTemperature);

    int pwmValue = (int)(P + I + D + FF);

    // MOSFET and Supply Safety Throttling (Simulated placeholders)
    float mosfetTemp = ambientRefTemp + (pwmValue / 10.0f); // Proxy for real MOSFET thermistor
    if (mosfetTemp > 70.0f) pwmValue *= 0.8f;
    if (mosfetTemp > 90.0f) pwmValue = 0;

    if (pwmValue > 200 && error > 1.0f) coolingHealth *= 0.999f;
    else if (fabsf(error) < 0.2f) coolingHealth = (coolingHealth * 0.999f) + 0.001f;

    analogWrite(coolerPin, constrain(pwmValue, 0, maxPWM));
    delay(200);
  }
}

void createCoolingProfile(float estimatedDewPoint, float ambientRefTemp) {
  for (int i = 0; i < 5; i++) { // Simplified for modular test
    float target = estimatedDewPoint + (2 - i);
    controlCoolingPWM(target, ambientRefTemp);
    delay(500);
  }
  stopCooling();
}

void verifyDewPointWithHeatingProfile() {
  sht4x.setHeater(SHT4X_HEATER_MED_100MS);
  for (int i = 0; i < 10; i++) {
    sensors_event_t h, t;
    sht4x.getEvent(&h, &t);
    delay(100);
  }
  sht4x.setHeater(SHT4X_NO_HEATER);
}

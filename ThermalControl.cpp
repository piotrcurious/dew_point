#include "ThermalControl.h"
#include "Sensors.h"
#include "Physics.h"

extern float coolingHealth;

void stopCooling() {
  extern int globalCurrentPWM;
  globalCurrentPWM = 0;
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

    // Hardware Safety Throttling
    // Gradual throttling to prevent supply oscillation (PD-like behavior)
    float supplyV = readSupplyVoltage();
    if (supplyV < 11.5f) {
        float throttle = (supplyV - 10.0f) / (11.5f - 10.0f); // 1.0 at 11.5V, 0.0 at 10.0V
        throttle = constrain(throttle, 0.0f, 1.0f);
        pwmValue = (int)((float)pwmValue * throttle);
    }

    if (pwmValue > 200 && error > 1.0f) coolingHealth *= 0.999f;
    else if (fabsf(error) < 0.2f) coolingHealth = (coolingHealth * 0.999f) + 0.001f;

    int finalPWM = constrain(pwmValue, 0, maxPWM);
    extern int globalCurrentPWM;
    globalCurrentPWM = finalPWM;
    analogWrite(coolerPin, finalPWM);
    delay(200);
  }
}

void createCoolingProfile(float estimatedDewPoint, float ambientRefTemp) {
  float targets[numCoolingPoints];
  for (int i = 0; i < 10; i++) targets[i] = estimatedDewPoint + (5 - i);
  float step = 2.0f / (numCoolingPoints - 10);
  for (int i = 10; i < numCoolingPoints; i++) targets[i] = estimatedDewPoint - 1.0f + (i - 10) * step;

  for (int i = 0; i < numCoolingPoints; i++) {
    controlCoolingPWM(targets[i], ambientRefTemp);
    delay(1000);

    float t = bme.readTemperature(), h = bme.readHumidity(), p = bme.readPressure() / 100.0f;
    float dp = adjustDewPointForPressure(calculateDewPoint(t, h), p);
    addRealTimeDataPoint(t, h, p, dp);

    Serial.print("Cooling ["); Serial.print(i + 1); Serial.print("/"); Serial.print(numCoolingPoints);
    Serial.print("] T="); Serial.print(t); Serial.print(" DP="); Serial.println(dp);
  }
  stopCooling();
}

void verifyDewPointWithHeatingProfile() {
  sht4x.setHeater(SHT4X_HEATER_MED_100MS);
  for (int i = 0; i < numHeatingPoints; i++) {
    sensors_event_t h, t;
    sht4x.getEvent(&h, &t);
    float p = bme.readPressure() / 100.0f;
    float dp = calculateDewPoint(t.temperature, h.relative_humidity);
    addRealTimeDataPoint(t.temperature, h.relative_humidity, p, dp);
    delay(200);
  }
  sht4x.setHeater(SHT4X_NO_HEATER);
}

#include "Physics.h"
#include <math.h>

float calculateDewPoint(float temperature, float humidity) {
  if (humidity <= 0.0f) humidity = 0.01f;
  float alpha = ((A * temperature) / (B + temperature)) + log(humidity / 100.0f);
  return (B * alpha) / (A - alpha);
}

float adjustDewPointForPressure(float dewPoint, float pressure) {
  // Linear approximation of the pressure correction for Magnus-Tetens.
  // This is physically sound for the expected range of 900-1100 hPa.
  // Delta T_dp ~ 0.001 K/hPa is a standard atmospheric approximation.
  return dewPoint + (pressure - 1013.25f) * pressureFactor;
}

float getTempAdsorptionFactor(float temperature, float ambientRefTemp, int currentPWM) {
  float deltaT = ambientRefTemp - temperature;
  if (deltaT < 0) deltaT = 0;

  // Airflow is proportional to cooling power (fan linked to PWM)
  // Higher airflow reduces adsorption concentration at the sensor surface
  float airflowFactor = 1.0f + ((float)currentPWM / 255.0f) * 2.0f;
  return 1.0f + (0.08f * deltaT) / airflowFactor;
}

float adjustDewPointForCO2(float dewPoint, float concentration, float temperature, float ambientRefTemp, int currentPWM) {
  return dewPoint * (1.0f + co2Factor * logf(1.0f + co2NonlinearCoeff * concentration) * getTempAdsorptionFactor(temperature, ambientRefTemp, currentPWM));
}

float adjustDewPointForSO2(float dewPoint, float concentration, float temperature, float ambientRefTemp, int currentPWM) {
  return dewPoint * (1.0f + so2Factor * powf(concentration, 2.0f) * so2NonlinearCoeff * getTempAdsorptionFactor(temperature, ambientRefTemp, currentPWM));
}

float adjustDewPointForNO2(float dewPoint, float concentration, float temperature, float ambientRefTemp, int currentPWM) {
  return dewPoint * (1.0f + no2Factor * (expf(no2NonlinearCoeff * concentration) - 1.0f) * getTempAdsorptionFactor(temperature, ambientRefTemp, currentPWM));
}

float removeContaminantEffect(float measuredDewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature, float ambientRefTemp, int currentPWM) {
  float f = getTempAdsorptionFactor(temperature, ambientRefTemp, currentPWM);
  float c = 1.0f + co2Factor * logf(1.0f + co2NonlinearCoeff * co2Dev) * f;
  float s = 1.0f + so2Factor * powf(so2Dev, 2.0f) * so2NonlinearCoeff * f;
  float n = 1.0f + no2Factor * (expf(no2NonlinearCoeff * no2Dev) - 1.0f) * f;
  return measuredDewPoint / (c * s * n);
}

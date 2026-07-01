#include "Physics.h"
#include <math.h>

float calculateDewPoint(float temperature, float humidity) {
  if (humidity <= 0.0f) humidity = 0.01f;
  float alpha = ((A * temperature) / (B + temperature)) + log(humidity / 100.0f);
  return (B * alpha) / (A - alpha);
}

float calculateEvaporationEnergy(float temperature, float humidity) {
  return 2500.8f - 2.36f * temperature;
}

float adjustDewPointForPressure(float dewPoint, float pressure) {
  return dewPoint + (pressure - 1013.25f) * pressureFactor;
}

float getTempAdsorptionFactor(float temperature, float ambientRefTemp, int currentPWM, float gasTempCoeff) {
  float deltaT = ambientRefTemp - temperature;
  if (deltaT < 0) deltaT = 0;
  float airflowFactor = 1.0f + ((float)currentPWM / 255.0f) * 2.0f;
  return (deltaT / airflowFactor);
}

float removeContaminantEffect(float measuredDewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature, float ambientRefTemp, int currentPWM) {
  float dT = ambientRefTemp - temperature;
  if (dT < 0) dT = 0;
  float flow = 1.0f + ((float)currentPWM / 255.0f) * 2.0f;

  // Differentiated Models for high contrast signatures:

  // 1. CO2: Logarithmic Surface Saturation
  // Signature: Slow rise, flattens out quickly
  float sigCO2 = co2Factor * logf(1.0f + co2NonlinearCoeff * co2Dev) * logf(1.0f + co2TempCoeff * dT / flow);

  // 2. SO2: Arrhenius Energetic Adsorption
  // Signature: Standard exponential rise
  float sigSO2 = so2Factor * powf(so2Dev, 2.0f) * so2NonlinearCoeff * (expf(so2TempCoeff * dT / flow) - 1.0f);

  // 3. NO2: Power-law Multi-layer
  // Signature: Accelerating rise (high curvature)
  float sigNO2 = no2Factor * (expf(no2NonlinearCoeff * no2Dev) - 1.0f) * powf(1.0f + no2TempCoeff * dT / flow, 1.5f);

  float dL = sigCO2 + sigSO2 + sigNO2;

  float L = calculateEvaporationEnergy(temperature, 100.0f);
  float T_k = temperature + 273.15f;
  float d_alpha = dL / (0.4615f * T_k);

  float alpha_meas = (A * measuredDewPoint) / (B + measuredDewPoint);
  float alpha_corr = alpha_meas - d_alpha;

  return (B * alpha_corr) / (A - alpha_corr);
}

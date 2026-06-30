#include "Physics.h"
#include <math.h>

float calculateDewPoint(float temperature, float humidity) {
  if (humidity <= 0.0f) humidity = 0.01f;
  float alpha = ((A * temperature) / (B + temperature)) + log(humidity / 100.0f);
  return (B * alpha) / (A - alpha);
}

float calculateEvaporationEnergy(float temperature, float humidity) {
  // Lv(T) = 2500.8 - 2.36 * T (T in Celsius, L in kJ/kg)
  return 2500.8f - 2.36f * temperature;
}

float adjustDewPointForPressure(float dewPoint, float pressure) {
  return dewPoint + (pressure - 1013.25f) * pressureFactor;
}

float getTempAdsorptionFactor(float temperature, float ambientRefTemp, int currentPWM, float gasTempCoeff) {
  float deltaT = ambientRefTemp - temperature;
  if (deltaT < 0) deltaT = 0;

  float airflowFactor = 1.0f + ((float)currentPWM / 255.0f) * 2.0f;
  // Arrhenius-like adsorption model: exp(E_a / RT)
  return expf((gasTempCoeff * deltaT) / airflowFactor);
}

float removeContaminantEffect(float measuredDewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature, float ambientRefTemp, int currentPWM) {
  float fc = getTempAdsorptionFactor(temperature, ambientRefTemp, currentPWM, co2TempCoeff);
  float fs = getTempAdsorptionFactor(temperature, ambientRefTemp, currentPWM, so2TempCoeff);
  float fn = getTempAdsorptionFactor(temperature, ambientRefTemp, currentPWM, no2TempCoeff);

  // Total evaporation energy shift (kJ/kg)
  float dL = (co2Factor * logf(1.0f + co2NonlinearCoeff * co2Dev) * fc) +
             (so2Factor * powf(so2Dev, 2.0f) * so2NonlinearCoeff * fs) +
             (no2Factor * (expf(no2NonlinearCoeff * no2Dev) - 1.0f) * fn);

  // Latent heat L in kJ/kg
  float L = calculateEvaporationEnergy(measuredDewPoint, 100.0f);

  // Clausius-Clapeyron derived shift:
  // dT = (Rv * T^2 / L) * (dL / L)
  // Rv for water vapor = 0.4615 kJ/(kg*K)
  float T_k = measuredDewPoint + 273.15f;
  float shift = (0.4615f * T_k * T_k / L) * (dL / L);

  return measuredDewPoint - shift;
}

#include "Physics.h"
#include <math.h>

float calculateDewPoint(float temperature, float humidity) {
  if (humidity <= 0.0f) humidity = 0.01f;
  float alpha = ((A * temperature) / (B + temperature)) + log(humidity / 100.0f);
  return (B * alpha) / (A - alpha);
}

float adjustDewPointForPressure(float dewPoint, float pressure) {
  return dewPoint + (pressure - 1013.25f) * pressureFactor;
}

float getTempAdsorptionFactor(float temperature, float ambientRefTemp) {
  float deltaT = ambientRefTemp - temperature;
  if (deltaT < 0) deltaT = 0;

  // Physics model: Adsorption is stronger at low temperatures and low airflow.
  // We assume airflow is proportional to cooling power (fan linked to PWM).
  // This helps distinguish contaminant effects from natural condensation.
  float airflowFactor = 1.0f; // Placeholder for real-time airflow sensor if available
  return 1.0f + (0.05f * deltaT) / airflowFactor;
}

float adjustDewPointForCO2(float dewPoint, float concentration, float temperature, float ambientRefTemp) {
  float adj = co2Factor * logf(1.0f + co2NonlinearCoeff * concentration) * getTempAdsorptionFactor(temperature, ambientRefTemp);
  return dewPoint + dewPoint * adj;
}

float adjustDewPointForSO2(float dewPoint, float concentration, float temperature, float ambientRefTemp) {
  float adj = so2Factor * powf(concentration, 2.0f) * so2NonlinearCoeff * getTempAdsorptionFactor(temperature, ambientRefTemp);
  return dewPoint + dewPoint * adj;
}

float adjustDewPointForNO2(float dewPoint, float concentration, float temperature, float ambientRefTemp) {
  float adj = no2Factor * (expf(no2NonlinearCoeff * concentration) - 1.0f) * getTempAdsorptionFactor(temperature, ambientRefTemp);
  return dewPoint + dewPoint * adj;
}

float removeContaminantEffect(float measuredDewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature, float ambientRefTemp) {
  float adj_co2 = co2Factor * logf(1.0f + co2NonlinearCoeff * co2Dev) * (1.0f + co2TempCoeff * (ambientRefTemp - temperature > 0 ? ambientRefTemp - temperature : 0));
  float adj_so2 = so2Factor * powf(so2Dev, 2.0f) * so2NonlinearCoeff * (1.0f + so2TempCoeff * (ambientRefTemp - temperature > 0 ? ambientRefTemp - temperature : 0));
  float adj_no2 = no2Factor * (expf(no2NonlinearCoeff * no2Dev) - 1.0f) * (1.0f + no2TempCoeff * (ambientRefTemp - temperature > 0 ? ambientRefTemp - temperature : 0));
  return measuredDewPoint / (1.0f + adj_co2 + adj_so2 + adj_no2);
}

#ifndef PHYSICS_H
#define PHYSICS_H

#include "config.h"

float calculateDewPoint(float temperature, float humidity);
float adjustDewPointForPressure(float dewPoint, float pressure);
float getTempAdsorptionFactor(float temperature, float ambientRefTemp, int currentPWM);
float adjustDewPointForCO2(float dewPoint, float concentration, float temperature, float ambientRefTemp, int currentPWM);
float adjustDewPointForSO2(float dewPoint, float concentration, float temperature, float ambientRefTemp, int currentPWM);
float adjustDewPointForNO2(float dewPoint, float concentration, float temperature, float ambientRefTemp, int currentPWM);
float removeContaminantEffect(float measuredDewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature, float ambientRefTemp, int currentPWM);

#endif

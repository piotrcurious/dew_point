#ifndef PHYSICS_H
#define PHYSICS_H

#include "config.h"

float calculateDewPoint(float temperature, float humidity);
float calculateEvaporationEnergy(float temperature, float humidity);
float adjustDewPointForPressure(float dewPoint, float pressure);
float getTempAdsorptionFactor(float temperature, float ambientRefTemp, int currentPWM, float gasTempCoeff);
float removeContaminantEffect(float measuredDewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature, float ambientRefTemp, int currentPWM);

#endif

#ifndef THERMAL_CONTROL_H
#define THERMAL_CONTROL_H

#include "config.h"

extern void addRealTimeDataPoint(float t, float h, float p, float dp);

void stopCooling();
void controlCoolingPWM(float targetTemperature, float ambientRefTemp);
void createCoolingProfile(float estimatedDewPoint, float ambientRefTemp);
void verifyDewPointWithHeatingProfile();

#endif

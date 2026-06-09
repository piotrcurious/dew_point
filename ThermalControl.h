#ifndef THERMAL_CONTROL_H
#define THERMAL_CONTROL_H

#include "config.h"

void stopCooling();
void controlCoolingPWM(float targetTemperature, float ambientRefTemp);
void createCoolingProfile(float estimatedDewPoint, float ambientRefTemp);
void verifyDewPointWithHeatingProfile();

#endif

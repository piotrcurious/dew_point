#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "config.h"

extern float empiricalTemperatures[];
extern float empiricalHumidities[];
extern float empiricalPressures[];
extern float rawDewPoints[];
extern float copyTemps[];
extern float copyHums[];
extern float copyPress[];
extern int   dataPointIndex;
extern bool  bufferFull;

void addRealTimeDataPoint(float t, float h, float p, float dp);
void monteCarloSimulation(float *empiricalTemps, float *empiricalHumidities, float *empiricalPressures, int n, int headIndex, float ambientRefTemp);

#endif

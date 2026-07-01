#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "config.h"
#include <Arduino.h>

extern float empiricalTemperatures[totalDataPoints];
extern float empiricalHumidities[totalDataPoints];
extern float empiricalPressures[totalDataPoints];
extern int   empiricalPWMs[totalDataPoints];
extern float rawDewPoints[totalDataPoints];
extern float copyTemps[totalDataPoints];
extern float copyHums[totalDataPoints];
extern float copyPress[totalDataPoints];
extern int   copyPWMs[totalDataPoints];
extern float adjDP_global[totalDataPoints];
extern float weights_global[totalDataPoints];
extern float vTemps_global[totalDataPoints];
extern bool bufferFull;
extern int dataPointIndex;

void addRealTimeDataPoint(float t, float h, float p, float dp, int pwm);
void monteCarloSimulation(float *empiricalTemps, float *empiricalHumidities, float *empiricalPressures, int *pwms, int n, int headIndex, float ambientRefTemp);

float objectiveFunction(float c, float s, float no, float *temps, float *hums, int *pwms, int n, int headIndex, float ambientRefTemp);
float computeWeightedVariance(float *data, float *weights, int n, float *meanOut);
float computeTemperatureCorrelation(float *dewPoints, float *temps, float *weights, int n);

#endif

#include "Optimization.h"
#include "Physics.h"
#include <algorithm>
#include <cmath>

float empiricalTemperatures[totalDataPoints];
float empiricalHumidities[totalDataPoints];
float empiricalPressures[totalDataPoints];
float rawDewPoints[totalDataPoints];
float copyTemps[totalDataPoints];
float copyHums[totalDataPoints];
float copyPress[totalDataPoints];
float adjDP_global[totalDataPoints];
float weights_global[totalDataPoints];
float vTemps_global[totalDataPoints];
int dataPointIndex = 0;
bool bufferFull = false;

extern SemaphoreHandle_t dataMutex;
extern SemaphoreHandle_t factorMutex;
extern float currentCO2Factor, currentSO2Factor, currentNO2Factor, currentConfidence;
extern void saveCalibration(float co2, float so2, float no2);

void addRealTimeDataPoint(float t, float h, float p, float dp) {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  empiricalTemperatures[dataPointIndex] = t;
  empiricalHumidities[dataPointIndex]   = h;
  empiricalPressures[dataPointIndex]    = p;
  dataPointIndex++;
  if (dataPointIndex >= totalDataPoints) {
    dataPointIndex = 0;
    bufferFull = true;
  }
  xSemaphoreGive(dataMutex);
}

float computeWeightedVariance(float *data, float *weights, int n, float *meanOut) {
  float totalWeight = 0, weightedSum = 0;
  for (int i = 0; i < n; i++) { weightedSum += data[i] * weights[i]; totalWeight += weights[i]; }
  if (totalWeight < 1e-6f) return 1e12f;
  float weightedMean = weightedSum / totalWeight;
  if (meanOut) *meanOut = weightedMean;
  float sumSq = 0;
  for (int i = 0; i < n; i++) { float d = data[i] - weightedMean; sumSq += weights[i] * d * d; }
  return sumSq / totalWeight;
}

float computeTemperatureCorrelation(float *dewPoints, float *temps, float *weights, int n) {
  float tw=0, st=0, sdp=0;
  for(int i=0; i<n; i++) { st += temps[i]*weights[i]; sdp += dewPoints[i]*weights[i]; tw += weights[i]; }
  if (tw < 1e-6f) return 0;
  float mt=st/tw, mdp=sdp/tw;
  float num=0, dt=0, ddp=0;
  for(int i=0; i<n; i++) {
    float t=temps[i]-mt, d=dewPoints[i]-mdp;
    num += weights[i]*t*d; dt += weights[i]*t*t; ddp += weights[i]*d*d;
  }
  if (dt < 1e-12 || ddp < 1e-12) return 0;
  return fabsf(num / sqrtf(dt * ddp));
}

bool isPointValid(int i, float *temps, float *hums, int n, int headIndex) {
  if (hums[i] > 98.0f || hums[i] < 2.0f) return false;
  int prev = (i - 1 + n) % n;
  if (n == totalDataPoints || i != (headIndex % n)) {
    float dT = fabsf(temps[i] - temps[prev]);
    float dH = fabsf(hums[i] - hums[prev]);
    if (dT > 8.0f || dH > 15.0f) return false;
  }
  return true;
}

struct SimplexPoint {
  float p[3];
  float f;
};

float objectiveFunction(float c, float s, float no, float *temps, float *hums, int n, int headIndex, float ambientRefTemp, int currentPWM) {
  int vp = 0;
  for (int j = 0; j < n; j++) {
    if (isPointValid(j, temps, hums, n, headIndex)) {
      adjDP_global[vp] = removeContaminantEffect(rawDewPoints[j], c, s, no, temps[j], ambientRefTemp, currentPWM);
      weights_global[vp] = 1.0f;
      vTemps_global[vp] = temps[j];
      vp++;
    }
  }
  if (vp < 10) return 1e20f;

  float var = computeWeightedVariance(adjDP_global, weights_global, vp, NULL);
  float corr = computeTemperatureCorrelation(adjDP_global, vTemps_global, weights_global, vp);

  float penalty = 0.0f;
  if (c < 0) penalty += 1e8f * (0.0f - c);
  if (s < 0) penalty += 1e8f * (0.0f - s);
  if (no < 0) penalty += 1e8f * (0.0f - no);
  if (c > 5) penalty += 1e8f * (c - 5.0f);
  if (s > 5) penalty += 1e8f * (s - 5.0f);
  if (no > 5) penalty += 1e8f * (no - 5.0f);

  return (var * 10000.0f) + (corr * 100000.0f) + (0.0001f * (c*c + s*s + no*no)) + penalty;
}

void nelderMead(float *bestC, float *bestS, float *bestN, float *bestErr, float *temps, float *hums, int n, int headIndex, float ambientRefTemp, int currentPWM) {
  SimplexPoint s[4];
  float step = 0.1f;
  s[0].p[0] = *bestC; s[0].p[1] = *bestS; s[0].p[2] = *bestN;
  s[1].p[0] = *bestC + step; s[1].p[1] = *bestS; s[1].p[2] = *bestN;
  s[2].p[0] = *bestC; s[2].p[1] = *bestS + step; s[2].p[2] = *bestN;
  s[3].p[0] = *bestC; s[3].p[1] = *bestS; s[3].p[2] = *bestN + step;

  for (int i = 0; i < 4; i++) s[i].f = objectiveFunction(s[i].p[0], s[i].p[1], s[i].p[2], temps, hums, n, headIndex, ambientRefTemp, currentPWM);

  const int maxIter = 500;
  for (int iter = 0; iter < maxIter; iter++) {
    std::sort(s, s + 4, [](const SimplexPoint &a, const SimplexPoint &b) { return a.f < b.f; });
    float coordDiff = fabsf(s[3].p[0] - s[0].p[0]) + fabsf(s[3].p[1] - s[0].p[1]) + fabsf(s[3].p[2] - s[0].p[2]);
    if (coordDiff < 1e-8f) break;

    float mid[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) { mid[0] += s[i].p[0] / 3.0f; mid[1] += s[i].p[1] / 3.0f; mid[2] += s[i].p[2] / 3.0f; }

    float ref[3];
    for (int i = 0; i < 3; i++) ref[i] = mid[i] + 1.0f * (mid[i] - s[3].p[i]);
    float refF = objectiveFunction(ref[0], ref[1], ref[2], temps, hums, n, headIndex, ambientRefTemp, currentPWM);

    if (s[0].f <= refF && refF < s[2].f) {
      s[3].p[0] = ref[0]; s[3].p[1] = ref[1]; s[3].p[2] = ref[2]; s[3].f = refF;
    } else if (refF < s[0].f) {
      float exp[3];
      for (int i = 0; i < 3; i++) exp[i] = mid[i] + 2.0f * (ref[i] - mid[i]);
      float expF = objectiveFunction(exp[0], exp[1], exp[2], temps, hums, n, headIndex, ambientRefTemp, currentPWM);
      if (expF < refF) { s[3].p[0] = exp[0]; s[3].p[1] = exp[1]; s[3].p[2] = exp[2]; s[3].f = expF; }
      else { s[3].p[0] = ref[0]; s[3].p[1] = ref[1]; s[3].p[2] = ref[2]; s[3].f = refF; }
    } else {
      float con[3];
      for (int i = 0; i < 3; i++) con[i] = mid[i] + 0.5f * (s[3].p[i] - mid[i]);
      float conF = objectiveFunction(con[0], con[1], con[2], temps, hums, n, headIndex, ambientRefTemp, currentPWM);
      if (conF < s[3].f) { s[3].p[0] = con[0]; s[3].p[1] = con[1]; s[3].p[2] = con[2]; s[3].f = conF; }
      else {
        for (int i = 1; i < 4; i++) {
          for (int j = 0; j < 3; j++) s[i].p[j] = s[0].p[j] + 0.5f * (s[i].p[j] - s[0].p[j]);
          s[i].f = objectiveFunction(s[i].p[0], s[i].p[1], s[i].p[2], temps, hums, n, headIndex, ambientRefTemp, currentPWM);
        }
      }
    }
  }
  *bestC = s[0].p[0]; *bestS = s[0].p[1]; *bestN = s[0].p[2]; *bestErr = s[0].f;
}

void monteCarloSimulation(float *empiricalTemps, float *empiricalHumidities, float *empiricalPressures, int n, int headIndex, float ambientRefTemp, int currentPWM) {
  if (n < 5) return;
  float bestError = 1e38, bestCO2 = currentCO2Factor, bestSO2 = currentSO2Factor, bestNO2 = currentNO2Factor;

  for (float c = 0; c <= 4.0f; c += 0.4f) {
    for (float s = 0; s <= 4.0f; s += 0.4f) {
      for (float no = 0; no <= 4.0f; no += 0.4f) {
        float err = objectiveFunction(c, s, no, empiricalTemps, empiricalHumidities, n, headIndex, ambientRefTemp, currentPWM);
        if (err < bestError) { bestError = err; bestCO2 = c; bestSO2 = s; bestNO2 = no; }
      }
    }
  }

  float fc = bestCO2, fs = bestSO2, fn = bestNO2;
  for (float c = std::max(0.0f, fc-0.4f); c <= std::min(5.0f, fc+0.4f); c += 0.1f) {
    for (float s = std::max(0.0f, fs-0.4f); s <= std::min(5.0f, fs+0.4f); s += 0.1f) {
      for (float no = std::max(0.0f, fn-0.4f); no <= std::min(5.0f, fn+0.4f); no += 0.1f) {
        float err = objectiveFunction(c, s, no, empiricalTemps, empiricalHumidities, n, headIndex, ambientRefTemp, currentPWM);
        if (err < bestError) { bestError = err; bestCO2 = c; bestSO2 = s; bestNO2 = no; }
      }
    }
  }

  nelderMead(&bestCO2, &bestSO2, &bestNO2, &bestError, empiricalTemps, empiricalHumidities, n, headIndex, ambientRefTemp, currentPWM);

  float confidence = 1.0f / (1.0f + bestError);
  xSemaphoreTake(factorMutex, portMAX_DELAY);
  currentCO2Factor = bestCO2; currentSO2Factor = bestSO2; currentNO2Factor = bestNO2; currentConfidence = confidence;
  saveCalibration(bestCO2, bestSO2, bestNO2);
  xSemaphoreGive(factorMutex);
}

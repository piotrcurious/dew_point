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

// Internal helpers for MC
float computeWeightedVariance(float *data, float *weights, int n) {
  float totalWeight = 0, weightedSum = 0;
  for (int i = 0; i < n; i++) { weightedSum += data[i] * weights[i]; totalWeight += weights[i]; }
  float weightedMean = weightedSum / totalWeight;
  float sumSq = 0;
  for (int i = 0; i < n; i++) { float d = data[i] - weightedMean; sumSq += weights[i] * d * d; }
  return sumSq / totalWeight;
}

float computeTemperatureCorrelation(float *dewPoints, float *temps, float *weights, int n) {
  float tw=0, st=0, sdp=0;
  for(int i=0; i<n; i++) { st += temps[i]*weights[i]; sdp += dewPoints[i]*weights[i]; tw += weights[i]; }
  float mt=st/tw, mdp=sdp/tw;
  float num=0, dt=0, ddp=0;
  for(int i=0; i<n; i++) {
    float t=temps[i]-mt, d=dewPoints[i]-mdp;
    num += weights[i]*t*d; dt += weights[i]*t*t; ddp += weights[i]*d*d;
  }
  if (dt < 1e-6 || ddp < 1e-6) return 0;
  return fabsf(num / sqrtf(dt * ddp));
}

bool isPointValid(int i, float *temps, float *hums, int n) {
  if (hums[i] > 95.0f || hums[i] < 5.0f) return false;
  if (i > 0) {
    float dT = fabsf(temps[i] - temps[i-1]);
    if (dT > 5.0f) return false; // Thermal gradient too high (noise)
  }
  return true;
}

struct SimplexPoint {
  float p[3]; // CO2, SO2, NO2
  float f;    // Cost
};

float objectiveFunction(float c, float s, float no, float *temps, float *hums, int n, float ambientRefTemp, int currentPWM) {
  int vp = 0;
  for (int j = 0; j < n; j++) {
    if (isPointValid(j, temps, hums, n)) {
      adjDP_global[vp] = removeContaminantEffect(rawDewPoints[j], c, s, no, temps[j], ambientRefTemp, currentPWM);
      int age = (dataPointIndex > 0) ? (dataPointIndex - 1 - j + n) % n : 0;
      weights_global[vp] = expf(-0.001f * age); // Slower decay for test stability
      vTemps_global[vp] = temps[j];
      vp++;
    }
  }
  if (vp < 2) return 1e6f;
  float var = computeWeightedVariance(adjDP_global, weights_global, vp);
  float corr = computeTemperatureCorrelation(adjDP_global, vTemps_global, weights_global, vp);
  float penalty = 0.0f;
  if (c < 0 || c > 10) penalty += 1000.0f;
  if (s < 0 || s > 10) penalty += 1000.0f;
  if (no < 0 || no > 10) penalty += 1000.0f;

  // Combined Error: minimize corrected DP variance and temperature cross-sensitivity.
  return var + (150.0f * corr) + 0.1f * (c + s + no) + penalty;
}

void nelderMead(float *bestC, float *bestS, float *bestN, float *bestErr, float *temps, float *hums, int n, float ambientRefTemp, int currentPWM) {
  SimplexPoint s[4];
  // Initial simplex centered around coarse best
  s[0].p[0] = *bestC; s[0].p[1] = *bestS; s[0].p[2] = *bestN;
  s[1].p[0] = *bestC + 2.0f; s[1].p[1] = *bestS; s[1].p[2] = *bestN;
  s[2].p[0] = *bestC; s[2].p[1] = *bestS + 2.0f; s[2].p[2] = *bestN;
  s[3].p[0] = *bestC; s[3].p[1] = *bestS; s[3].p[2] = *bestN + 2.0f;

  for (int i = 0; i < 4; i++) s[i].f = objectiveFunction(s[i].p[0], s[i].p[1], s[i].p[2], temps, hums, n, ambientRefTemp, currentPWM);

  for (int iter = 0; iter < 60; iter++) {
    // Sort
    std::sort(s, s + 4, [](const SimplexPoint &a, const SimplexPoint &b) { return a.f < b.f; });

    // Termination check
    if (fabsf(s[3].f - s[0].f) < 0.0001f) break;

    // Centroid of best 3
    float mid[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) { mid[0] += s[i].p[0] / 3.0f; mid[1] += s[i].p[1] / 3.0f; mid[2] += s[i].p[2] / 3.0f; }

    // Reflection
    float ref[3];
    for (int i = 0; i < 3; i++) ref[i] = mid[i] + 1.0f * (mid[i] - s[3].p[i]);
    float refF = objectiveFunction(ref[0], ref[1], ref[2], temps, hums, n, ambientRefTemp, currentPWM);

    if (s[0].f <= refF && refF < s[2].f) {
      s[3].p[0] = ref[0]; s[3].p[1] = ref[1]; s[3].p[2] = ref[2]; s[3].f = refF;
    } else if (refF < s[0].f) {
      // Expansion
      float exp[3];
      for (int i = 0; i < 3; i++) exp[i] = mid[i] + 2.0f * (ref[i] - mid[i]);
      float expF = objectiveFunction(exp[0], exp[1], exp[2], temps, hums, n, ambientRefTemp, currentPWM);
      if (expF < refF) { s[3].p[0] = exp[0]; s[3].p[1] = exp[1]; s[3].p[2] = exp[2]; s[3].f = expF; }
      else { s[3].p[0] = ref[0]; s[3].p[1] = ref[1]; s[3].p[2] = ref[2]; s[3].f = refF; }
    } else {
      // Contraction
      float con[3];
      for (int i = 0; i < 3; i++) con[i] = mid[i] + 0.5f * (s[3].p[i] - mid[i]);
      float conF = objectiveFunction(con[0], con[1], con[2], temps, hums, n, ambientRefTemp, currentPWM);
      if (conF < s[3].f) { s[3].p[0] = con[0]; s[3].p[1] = con[1]; s[3].p[2] = con[2]; s[3].f = conF; }
      else {
        // Shrink
        for (int i = 1; i < 4; i++) {
          for (int j = 0; j < 3; j++) s[i].p[j] = s[0].p[j] + 0.5f * (s[i].p[j] - s[0].p[j]);
          s[i].f = objectiveFunction(s[i].p[0], s[i].p[1], s[i].p[2], temps, hums, n, ambientRefTemp, currentPWM);
        }
      }
    }
  }
  *bestC = s[0].p[0]; *bestS = s[0].p[1]; *bestN = s[0].p[2]; *bestErr = s[0].f;
}

void monteCarloSimulation(float *empiricalTemps, float *empiricalHumidities, float *empiricalPressures, int n, int headIndex, float ambientRefTemp, int currentPWM) {
  if (n < 5) return; // Need minimum data points
  float bestError = 1e6, bestCO2 = currentCO2Factor, bestSO2 = currentSO2Factor, bestNO2 = currentNO2Factor;

  auto search = [&](float cS, float cE, float cStep, float sS, float sE, float sStep, float nS, float nE, float nStep) {
    for (float c = cS; c <= cE + 0.001f; c += cStep) {
      for (float s = sS; s <= sE + 0.001f; s += sStep) {
        for (float no = nS; no <= nE + 0.001f; no += nStep) {
          int vp = 0;
          for (int j = 0; j < n; j++) {
            if (isPointValid(j, empiricalTemps, empiricalHumidities, n)) {
              adjDP_global[vp] = removeContaminantEffect(rawDewPoints[j], c, s, no, empiricalTemps[j], ambientRefTemp, currentPWM);
              // Chronological weighting: more recent points have higher weight
              int age = (headIndex - 1 - j + n) % n;
              weights_global[vp] = expf(-0.002f * age);
              vTemps_global[vp] = empiricalTemps[j];
              vp++;
            }
          }
          if (vp < 2) continue;
          float var = computeWeightedVariance(adjDP_global, weights_global, vp);
          float corr = computeTemperatureCorrelation(adjDP_global, vTemps_global, weights_global, vp);
          float err = (100.0f * corr) + var + 0.05f * (c + s + no);
          if (err < bestError) { bestError = err; bestCO2 = c; bestSO2 = s; bestNO2 = no; }
        }
      }
    }
  };

  // Pass 1: Coarse Grid Search to find global neighborhood
  search(0, 5, 1.0f, 0, 5, 1.0f, 0, 5, 1.0f);

  // Pass 2: Nelder-Mead Simplex for high-precision refinement
  nelderMead(&bestCO2, &bestSO2, &bestNO2, &bestError, empiricalTemps, empiricalHumidities, n, ambientRefTemp, currentPWM);

  float confidence = 1.0f / (1.0f + bestError);
  xSemaphoreTake(factorMutex, portMAX_DELAY);
  if (confidence > currentConfidence * 1.01f || currentConfidence < 0.1f) {
    currentCO2Factor = bestCO2; currentSO2Factor = bestSO2; currentNO2Factor = bestNO2; currentConfidence = confidence;
    saveCalibration(bestCO2, bestSO2, bestNO2);
  }
  xSemaphoreGive(factorMutex);
}

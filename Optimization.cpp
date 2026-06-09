#include "Optimization.h"
#include "Physics.h"

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

void monteCarloSimulation(float *empiricalTemps, float *empiricalHumidities, float *empiricalPressures, int n, int headIndex, float ambientRefTemp, int currentPWM) {
  if (n == 0) return;
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

  // Pass 1: Coarse
  search(0, 5, 1.0, 0, 5, 1.0, 0, 5, 1.0);
  // Pass 2: Medium
  search(max(0.0f, bestCO2-0.5f), min(5.0f, bestCO2+0.5f), 0.2f, max(0.0f, bestSO2-0.5f), min(5.0f, bestSO2+0.5f), 0.2f, max(0.0f, bestNO2-0.5f), min(5.0f, bestNO2+0.5f), 0.2f);
  // Pass 3: Fine
  search(max(0.0f, bestCO2-0.1f), min(5.0f, bestCO2+0.1f), 0.05f, max(0.0f, bestSO2-0.1f), min(5.0f, bestSO2+0.1f), 0.05f, max(0.0f, bestNO2-0.1f), min(5.0f, bestNO2+0.1f), 0.05f);

  float confidence = 1.0f / (1.0f + bestError);
  xSemaphoreTake(factorMutex, portMAX_DELAY);
  if (confidence > currentConfidence * 1.05f || currentConfidence < 0.1f) {
    currentCO2Factor = bestCO2; currentSO2Factor = bestSO2; currentNO2Factor = bestNO2; currentConfidence = confidence;
    saveCalibration(bestCO2, bestSO2, bestNO2);
  }
  xSemaphoreGive(factorMutex);
}

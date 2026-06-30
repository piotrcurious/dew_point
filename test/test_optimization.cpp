#include "arduino_mock.h"
#include "../config.h"
#include "../Physics.h"
#include "../Optimization.h"
#include <iostream>
#include <cassert>
#include <vector>
#include <iomanip>
#include <random>

Adafruit_BME280 bme;
Adafruit_SHT4x sht4x;
SemaphoreHandle_t dataMutex = xSemaphoreCreateMutex();
SemaphoreHandle_t factorMutex = xSemaphoreCreateMutex();

float currentCO2Factor = 1.0f;
float currentSO2Factor = 1.0f;
float currentNO2Factor = 1.0f;
float currentConfidence = 0.0f;
float coolingHealth = 1.0f;

void saveCalibration(float co2, float so2, float no2) {
    currentCO2Factor = co2;
    currentSO2Factor = so2;
    currentNO2Factor = no2;
}

// Helper to simulate data with physically grounded shift
void simulateData(float targetCO2, float targetSO2, float targetNO2) {
    float ambientT = 25.0f;
    float ambientP = 1013.25f;
    float trueDP = 10.0f;

    std::default_random_engine generator;
    std::normal_distribution<float> noise(0.0, 0.01);

    for (int i = 0; i < totalDataPoints; i++) {
        float temp = ambientT - (i * 0.05f);
        if (temp < 0.0f) temp = 0.0f;

        float fc = getTempAdsorptionFactor(temp, ambientT, 0, co2TempCoeff);
        float fs = getTempAdsorptionFactor(temp, ambientT, 0, so2TempCoeff);
        float fn = getTempAdsorptionFactor(temp, ambientT, 0, no2TempCoeff);

        float dL = (co2Factor * logf(1.0f + co2NonlinearCoeff * targetCO2) * fc) +
                   (so2Factor * powf(targetSO2, 2.0f) * so2NonlinearCoeff * fs) +
                   (no2Factor * (expf(no2NonlinearCoeff * targetNO2) - 1.0f) * fn);

        float L = 2500.8f - 2.36f * trueDP;
        float T_k = trueDP + 273.15f;
        float shift = (0.4615f * T_k * T_k / L) * (dL / L);

        float measuredDP = trueDP + shift + noise(generator);

        float alphaMeasured = (A * measuredDP) / (B + measuredDP);
        float hContam = 100.0f * expf(alphaMeasured - (A * temp) / (B + temp));

        empiricalTemperatures[i] = temp;
        empiricalHumidities[i] = hContam;
        empiricalPressures[i] = ambientP;
    }
    bufferFull = true;
}

void runTest(float tC, float tS, float tN, const char* name) {
    std::cout << "\n--- Test: " << name << " ---" << std::endl;
    simulateData(tC, tS, tN);
    for (int i = 0; i < totalDataPoints; i++) {
        float dp = calculateDewPoint(empiricalTemperatures[i], empiricalHumidities[i]);
        rawDewPoints[i] = adjustDewPointForPressure(dp, empiricalPressures[i]);
    }
    monteCarloSimulation(empiricalTemperatures, empiricalHumidities, empiricalPressures, totalDataPoints, totalDataPoints, 25.0f, 0);

    std::cout << "Target: [" << tC << ", " << tS << ", " << tN << "]" << std::endl;
    std::cout << "Found:  [" << currentCO2Factor << ", " << currentSO2Factor << ", " << currentNO2Factor << "]" << std::endl;
    std::cout << "Confidence: " << currentConfidence << std::endl;

    if (fabs(currentCO2Factor - tC) < 0.25f && fabs(currentSO2Factor - tS) < 0.25f && fabs(currentNO2Factor - tN) < 0.25f) {
        std::cout << "RESULT: SUCCESS" << std::endl;
    } else {
        std::cout << "RESULT: FAILURE" << std::endl;
    }
}

int main() {
    std::cout << std::fixed << std::setprecision(4);
    runTest(1.0, 0.0, 0.0, "Pure CO2");
    runTest(0.0, 1.0, 0.0, "Pure SO2");
    runTest(0.0, 0.0, 1.0, "Pure NO2");
    runTest(1.5, 0.5, 0.7, "Mixed Complex");
    return 0;
}

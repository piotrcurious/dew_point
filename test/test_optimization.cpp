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

float currentCO2Factor = 0.0f;
float currentSO2Factor = 0.0f;
float currentNO2Factor = 0.0f;
float currentConfidence = 0.0f;
float coolingHealth = 1.0f;

void saveCalibration(float co2, float so2, float no2) {
    currentCO2Factor = co2;
    currentSO2Factor = so2;
    currentNO2Factor = no2;
}

// Helper to simulate data consistent with the NEW differentiated Physics.cpp
void simulateData(float targetCO2, float targetSO2, float targetNO2) {
    float ambientT = 25.0f;
    float ambientP = 1013.25f;
    float trueDP = 10.0f;

    std::default_random_engine generator;
    std::normal_distribution<float> noise(0.0, 0.005);

    for (int i = 0; i < totalDataPoints; i++) {
        float temp = ambientT - (i * 0.05f);
        if (temp < 0.0f) temp = 0.0f;
        int pwm = (i < 250) ? 50 : 200;
        float flow = 1.0f + ((float)pwm / 255.0f) * 2.0f;
        float dT = ambientT - temp;

        float sigCO2 = co2Factor * logf(1.0f + co2NonlinearCoeff * targetCO2) * logf(1.0f + co2TempCoeff * dT / flow);
        float sigSO2 = so2Factor * powf(targetSO2, 2.0f) * so2NonlinearCoeff * (expf(so2TempCoeff * dT / flow) - 1.0f);
        float sigNO2 = no2Factor * (expf(no2NonlinearCoeff * targetNO2) - 1.0f) * powf(1.0f + no2TempCoeff * dT / flow, 1.5f);

        float dL = sigCO2 + sigSO2 + sigNO2;
        float T_k = temp + 273.15f;
        float d_alpha = dL / (0.4615f * T_k);

        float alpha_true = (A * trueDP) / (B + trueDP);
        float alpha_meas = alpha_true + d_alpha;

        float measuredDP = (B * alpha_meas) / (A - alpha_meas) + noise(generator);

        float alphaMeasuredFinal = (A * measuredDP) / (B + measuredDP);
        float hContam = 100.0f * expf(alphaMeasuredFinal - (A * temp) / (B + temp));

        empiricalTemperatures[i] = temp;
        empiricalHumidities[i] = hContam;
        empiricalPressures[i] = ambientP;
        empiricalPWMs[i] = pwm;
    }
    bufferFull = true;
}

void runTest(float tC, float tS, float tN, const char* name) {
    std::cout << "\n--- Test: " << name << " ---" << std::endl;
    currentCO2Factor = 0; currentSO2Factor = 0; currentNO2Factor = 0;

    simulateData(tC, tS, tN);
    for (int i = 0; i < totalDataPoints; i++) {
        float dp = calculateDewPoint(empiricalTemperatures[i], empiricalHumidities[i]);
        rawDewPoints[i] = adjustDewPointForPressure(dp, empiricalPressures[i]);
    }
    monteCarloSimulation(empiricalTemperatures, empiricalHumidities, empiricalPressures, empiricalPWMs, totalDataPoints, totalDataPoints, 25.0f);

    std::cout << "Target: [" << tC << ", " << tS << ", " << tN << "]" << std::endl;
    std::cout << "Found:  [" << currentCO2Factor << ", " << currentSO2Factor << ", " << currentNO2Factor << "]" << std::endl;

    float diffC = fabs(currentCO2Factor - tC);
    float diffS = fabs(currentSO2Factor - tS);
    float diffN = fabs(currentNO2Factor - tN);

    if (diffC < 0.2f && diffS < 0.2f && diffN < 0.2f) {
        std::cout << "RESULT: SUCCESS" << std::endl;
    } else {
        std::cout << "RESULT: FAILURE (Diffs: " << diffC << ", " << diffS << ", " << diffN << ")" << std::endl;
    }
}

int main() {
    std::cout << std::fixed << std::setprecision(4);
    runTest(1.0, 0.0, 0.0, "Pure CO2");
    runTest(0.0, 1.0, 0.0, "Pure SO2");
    runTest(0.0, 0.0, 1.0, "Pure NO2");
    runTest(1.2, 0.5, 0.8, "Mixed Complex (Differentiated)");
    return 0;
}

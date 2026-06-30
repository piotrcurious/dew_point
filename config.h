#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ── Magnus-Tetens constants ───────────────────────────────────────────────────
const float A             = 17.27f;
const float B             = 237.7f;
const float pressureFactor = 0.001f;   // °C / hPa deviation from std pressure

// ── Non-linear contaminant-gas adjustment factors (Energy shift kJ/kg) ───────
const float co2Factor         = 12.0f;    // Logarithmic scale (CO₂)
const float so2Factor         = 45.0f;    // Polynomial scale  (SO₂)
const float no2Factor         = 30.0f;    // Exponential scale (NO₂)
const float co2NonlinearCoeff = 0.4f;
const float so2NonlinearCoeff = 0.9f;
const float no2NonlinearCoeff = 1.2f;

const float co2TempCoeff = 0.015f;
const float so2TempCoeff = 0.060f;
const float no2TempCoeff = 0.140f;

// ── PWM / pin settings ───────────────────────────────────────────────────────
const int   coolerPin         = 16;
const int   heaterPin         = 17;
const int   batteryAdcPin     = 34;
const int   supplyAdcPin      = 35;

// ADC Voltage Divider Ratios ( (R1 + R2) / R2 )
const float batteryDividerRatio = (10.0f + 2.2f) / 2.2f;
const float supplyDividerRatio  = (10.0f + 2.2f) / 2.2f;
const float adcRefVoltage       = 3.3f;
const int   adcResolution       = 4095;
const int   maxPWM            = 255;
const int   minPWM            = 50;
const int   numCoolingPoints  = 100;
const int   numHeatingPoints  = 50;
const int   totalDataPoints   = 500;

// Safety timeout for the cooling control loop (5 minutes).
const unsigned long coolingTimeoutMs = 300000UL;

// ── Filtering Constants ──────────────────────────────────────────────────────
const float emaAlphaT = 0.2f;
const float emaAlphaH = 0.1f;
const float emaAlphaP = 0.5f;

// ── PID Control constants ────────────────────────────────────────────────────
const float Kp = 40.0f;
const float Ki = 1.0f;
const float Kd = 20.0f;
const float Kf = 5.0f;

#endif

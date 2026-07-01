#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ── Magnus-Tetens constants ───────────────────────────────────────────────────
const float A             = 17.27f;
const float B             = 237.7f;
const float pressureFactor = 0.001f;   // °C / hPa deviation from std pressure

// ── Non-linear contaminant-gas adjustment factors ────────────────────────────
// Goals:
// 1. Normalized so dev=1.0 => dL ~ 20 kJ/kg at dT=15, flow=1.5
// 2. High mathematical contrast (Log vs Exp vs Power)

const float co2Factor         = 18.0f;
const float so2Factor         = 13.0f;
const float no2Factor         = 0.35f;

const float co2NonlinearCoeff = 0.5f;
const float so2NonlinearCoeff = 0.9f;
const float no2NonlinearCoeff = 1.0f;

// Gas specific temperature-signature coefficients
const float co2TempCoeff = 3.0f;    // Log(1 + 3*dT/flow)
const float so2TempCoeff = 0.12f;   // Exp(0.12*dT/flow) - 1
const float no2TempCoeff = 0.8f;    // (1 + 0.8*dT/flow)^1.5

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

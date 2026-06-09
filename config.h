#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ── Magnus-Tetens constants ───────────────────────────────────────────────────
const float A             = 17.27f;
const float B             = 237.7f;
const float pressureFactor = 0.001f;   // °C / hPa deviation from std pressure

// ── Non-linear contaminant-gas adjustment factors ────────────────────────────
const float co2Factor         = 0.025f;   // Logarithmic scale (CO₂)
const float so2Factor         = 0.08f;    // Polynomial scale  (SO₂)
const float no2Factor         = 0.05f;    // Exponential scale (NO₂)
const float co2NonlinearCoeff = 0.4f;
const float so2NonlinearCoeff = 0.9f;
const float no2NonlinearCoeff = 1.2f;

const float co2TempCoeff = 0.02f;
const float so2TempCoeff = 0.05f;
const float no2TempCoeff = 0.08f;

// ── PWM / pin settings ───────────────────────────────────────────────────────
const int   coolerPin         = 16;
const int   heaterPin         = 17;
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

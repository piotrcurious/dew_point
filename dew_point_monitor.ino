#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT4x.h>
#include <math.h>

// ── Sensor instances ─────────────────────────────────────────────────────────
Adafruit_BME280 bme;
Adafruit_SHT4x  sht4x;

// ── Magnus-Tetens constants ───────────────────────────────────────────────────
const float A             = 17.27f;
const float B             = 237.7f;
const float pressureFactor = 0.001f;   // °C / hPa deviation from std pressure

// ── Non-linear contaminant-gas adjustment factors ────────────────────────────
const float co2Factor         = 0.0025f;  // Logarithmic scale (CO₂)
const float so2Factor         = 0.008f;   // Polynomial scale  (SO₂)
const float no2Factor         = 0.005f;   // Exponential scale (NO₂)
const float co2NonlinearCoeff = 0.4f;
const float so2NonlinearCoeff = 0.9f;
const float no2NonlinearCoeff = 1.2f;

// ── PWM / pin settings ───────────────────────────────────────────────────────
const int   coolerPin         = 9;
const int   heaterPin         = 8;        // Reserved for external heater
const int   maxPWM            = 255;
const int   minPWM            = 50;
const int   totalDataPoints   = 19;

// FIX 3: Safety timeout for the cooling control loop (2 minutes).
const unsigned long coolingTimeoutMs = 120000UL;

// ── Empirical data storage ───────────────────────────────────────────────────
float empiricalDewPoints[totalDataPoints];
float empiricalTemperatures[totalDataPoints];
int   dataPointIndex = 0;

// ── Function prototypes ───────────────────────────────────────────────────────
// FIX 6: Added missing prototypes for initializeSensors() and
//        estimateInitialDewPoint(), which are called before their definitions.
// FIX 7: Removed the orphaned adjustDewPointForGasNonlinear() prototype
//        (declared but never defined — replaced by the three per-gas functions).
void  initializeSensors();
float estimateInitialDewPoint();
float calculateDewPoint(float temperature, float humidity);
float adjustDewPointForPressure(float dewPoint, float pressure);
float adjustDewPointForCO2(float dewPoint, float concentration);
float adjustDewPointForSO2(float dewPoint, float concentration);
float adjustDewPointForNO2(float dewPoint, float concentration);
float adjustDewPointForContaminants(float dewPoint, float co2Dev, float so2Dev, float no2Dev);
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax);
void  stopCooling();
void  controlCoolingPWM(float targetTemperature);
void  createCoolingProfile(float estimatedDewPoint);
void  verifyDewPointWithHeatingProfile();
float computeRMSE(float *simulated, float *empirical, int n);
float fitPolynomialCurve(float *x, float *y, int n);
void  monteCarloSimulation(float *empiricalTemps, float *empiricalDPs, int n);

// ── setup() ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  initializeSensors();

  float estimatedDewPoint = estimateInitialDewPoint();
  createCoolingProfile(estimatedDewPoint);
  verifyDewPointWithHeatingProfile();

  if (dataPointIndex > 1) {
    monteCarloSimulation(empiricalTemperatures, empiricalDewPoints, dataPointIndex);
  } else {
    Serial.println("Insufficient data points for Monte Carlo simulation.");
  }
}

void loop() {
  delay(5000);
}

// ── Sensor initialisation ────────────────────────────────────────────────────
void initializeSensors() {
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found — check wiring!");
    while (1);
  }
  if (!sht4x.begin()) {
    Serial.println("SHT4x not found — check wiring!");
    while (1);
  }

  sht4x.setPrecision(SHT4X_HIGH_PRECISION);
  sht4x.setHeater(SHT4X_NO_HEATER);   // Ensure heater is off at boot

  pinMode(coolerPin, OUTPUT);
  analogWrite(coolerPin, 0);
  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW);
}

// ── Initial dew point estimate ───────────────────────────────────────────────
float estimateInitialDewPoint() {
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  return calculateDewPoint(t, h);
}

// ── Magnus-Tetens dew point ──────────────────────────────────────────────────
float calculateDewPoint(float temperature, float humidity) {
  if (humidity <= 0.0f) humidity = 0.01f;  // Guard against log(0)
  float alpha = ((A * temperature) / (B + temperature)) + log(humidity / 100.0f);
  return (B * alpha) / (A - alpha);
}

// ── Pressure correction ──────────────────────────────────────────────────────
float adjustDewPointForPressure(float dewPoint, float pressure) {
  return dewPoint + (pressure - 1013.25f) * pressureFactor;
}

// ── Per-gas non-linear adjustments ──────────────────────────────────────────
float adjustDewPointForCO2(float dewPoint, float concentration) {
  float adj = co2Factor * log(1.0f + co2NonlinearCoeff * concentration);
  return dewPoint + dewPoint * adj;
}

float adjustDewPointForSO2(float dewPoint, float concentration) {
  float adj = so2Factor * powf(concentration, 2.0f) * so2NonlinearCoeff;
  return dewPoint + dewPoint * adj;
}

float adjustDewPointForNO2(float dewPoint, float concentration) {
  float adj = no2Factor * expf(no2NonlinearCoeff * concentration);
  return dewPoint + dewPoint * adj;
}

// ── Combined contaminant adjustment ─────────────────────────────────────────
float adjustDewPointForContaminants(float dewPoint, float co2Dev, float so2Dev, float no2Dev) {
  dewPoint = adjustDewPointForCO2(dewPoint, co2Dev);
  dewPoint = adjustDewPointForSO2(dewPoint, so2Dev);
  return    adjustDewPointForNO2(dewPoint, no2Dev);
}

// ── FIX 4: Float-safe map ────────────────────────────────────────────────────
// Arduino's built-in map() is integer-only; float inputs get silently
// truncated.  This replacement handles float ranges correctly.
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax) {
  if (fabsf(inMax - inMin) < 1e-9f) return outMin;
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// ── FIX 1: Dedicated cooling-off helper ─────────────────────────────────────
// The original code called controlCoolingPWM(0) to "turn off" the cooler,
// but that function treats its argument as a *target temperature* — passing 0
// would cause it to try to cool the surface to 0 °C (or loop indefinitely).
void stopCooling() {
  analogWrite(coolerPin, 0);
}

// ── Adaptive cooling with feedback ──────────────────────────────────────────
void controlCoolingPWM(float targetTemperature) {
  analogWrite(coolerPin, maxPWM);

  // FIX 3: Safety timeout — exit if target unreachable within 2 minutes.
  unsigned long startMs = millis();

  while (true) {
    if (millis() - startMs > coolingTimeoutMs) {
      Serial.println("WARNING: Cooling timeout — target temperature not reached.");
      break;
    }

    float currentTemp = bme.readTemperature();
    if (fabsf(currentTemp - targetTemperature) < 0.1f) break;

    // FIX 4: Use mapFloat() — Arduino map() is integer-only.
    int pwmValue = (int)mapFloat(currentTemp,
                                 targetTemperature - 5.0f, targetTemperature + 5.0f,
                                 (float)maxPWM, (float)minPWM);
    analogWrite(coolerPin, constrain(pwmValue, minPWM, maxPWM));
    delay(200);
  }
}

// ── Cooling profile (empirical data collection) ──────────────────────────────
void createCoolingProfile(float estimatedDewPoint) {
  float targetTemperatures[totalDataPoints];

  // FIX 2: Bracketing loop produced dp-5, dp+5, dp+5 (indices 0,1,2 all used
  // the same "+5" branch except index 0).  Corrected to dp-5, dp, dp+5
  // by using (i - 1) * 5 so the three bracket points are evenly spaced.
  for (int i = 0; i < 3; i++) {
    targetTemperatures[i] = estimatedDewPoint + (i - 1) * 5.0f;
  }
  // 16 fine steps from (dp - 1.0) to (dp + 0.875) at 0.125 °C increments
  for (int i = 3; i < totalDataPoints; i++) {
    targetTemperatures[i] = estimatedDewPoint - 1.0f + (i - 3) * 0.125f;
  }

  for (int i = 0; i < totalDataPoints; i++) {
    controlCoolingPWM(targetTemperatures[i]);
    delay(1000);  // Stabilisation pause

    float temp     = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0f;

    float dp = calculateDewPoint(temp, humidity);
    dp = adjustDewPointForPressure(dp, pressure);

    empiricalTemperatures[dataPointIndex] = temp;
    empiricalDewPoints[dataPointIndex]    = dp;
    dataPointIndex++;

    Serial.print("Cooling [");
    Serial.print(i + 1);   Serial.print("/"); Serial.print(totalDataPoints);
    Serial.print("] T=");   Serial.print(temp, 2);     Serial.print(" °C");
    Serial.print("  RH=");  Serial.print(humidity, 1); Serial.print(" %");
    Serial.print("  DP=");  Serial.print(dp, 2);       Serial.println(" °C");
  }

  // FIX 1: Use stopCooling() instead of controlCoolingPWM(0).
  stopCooling();
}

// ── Heating verification (SHT4x internal heater) ────────────────────────────
void verifyDewPointWithHeatingProfile() {
  const int heatingDataPoints = 10;
  float     heatingDewPoints[heatingDataPoints];

  // FIX 5: The original code used the non-existent SHT4X_HEATER_MEDIUM enum
  // and a two-argument form of setHeater() that doesn't exist in the Adafruit
  // library.  The correct approach:
  //   • Duration is encoded in the enum (SHT4X_HEATER_MED_100MS = medium
  //     power, 100 ms pulse).
  //   • The heater fires automatically on each getEvent() call.
  //   • SHT4X_NO_HEATER (single argument) disables it afterwards.
  sht4x.setHeater(SHT4X_HEATER_MED_100MS);

  float totalCondensationEffect = 0.0f;

  for (int i = 0; i < heatingDataPoints; i++) {
    sensors_event_t humEvent, tempEvent;
    sht4x.getEvent(&humEvent, &tempEvent);  // Heater pulse fires here

    float t = tempEvent.temperature;
    float h = humEvent.relative_humidity;
    heatingDewPoints[i] = calculateDewPoint(t, h);

    Serial.print("Heating [");
    Serial.print(i + 1);   Serial.print("/"); Serial.print(heatingDataPoints);
    Serial.print("] T=");   Serial.print(t, 2);  Serial.print(" °C");
    Serial.print("  RH=");  Serial.print(h, 1);  Serial.print(" %");
    Serial.print("  DP=");  Serial.print(heatingDewPoints[i], 2); Serial.println(" °C");

    delay(200);
  }

  // FIX 5 (cont.): Disable heater with the correct single-argument call.
  sht4x.setHeater(SHT4X_NO_HEATER);

  // Sum negative dew-point deltas as a proxy for condensation onset
  for (int i = 1; i < heatingDataPoints; i++) {
    float delta = heatingDewPoints[i] - heatingDewPoints[i - 1];
    if (delta < 0.0f) totalCondensationEffect += -delta;
  }

  Serial.print("Total non-linear condensation effect: ");
  Serial.print(totalCondensationEffect, 3);
  Serial.println(" °C");
}

// ── FIX 9 helper: RMSE between two float arrays ──────────────────────────────
// The original Monte Carlo used fitPolynomialCurve(empiricalTemps, simDPs, n)
// as its error metric, which fit a curve to the *simulated* data alone and
// never referenced empiricalDewPoints at all.  RMSE between simulated and
// empirical dew points is the correct comparison.
float computeRMSE(float *simulated, float *empirical, int n) {
  if (n == 0) return 1e6f;
  float sumSq = 0.0f;
  for (int i = 0; i < n; i++) {
    float d = simulated[i] - empirical[i];
    sumSq += d * d;
  }
  return sqrtf(sumSq / n);
}

// ── Quadratic least-squares fit — returns total absolute residual ─────────────
// (Kept as a standalone diagnostic; no longer misused as the Monte Carlo
//  error metric.)
float fitPolynomialCurve(float *x, float *y, int n) {
  if (n < 3) return 1e6f;

  float sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
  float sumY = 0, sumXY = 0, sumX2Y = 0;

  for (int i = 0; i < n; i++) {
    float xi  = x[i], yi = y[i];
    float xi2 = xi * xi;
    sumX   += xi;
    sumX2  += xi2;
    sumX3  += xi2 * xi;
    sumX4  += xi2 * xi2;
    sumY   += yi;
    sumXY  += xi * yi;
    sumX2Y += xi2 * yi;
  }

  float D = n    * (sumX2 * sumX4 - sumX3 * sumX3)
          - sumX * (sumX  * sumX4 - sumX2 * sumX3)
          + sumX2* (sumX  * sumX3 - sumX2 * sumX2);
  if (fabsf(D) < 1e-12f) return 1e6f;

  float a = (sumY  * (sumX2 * sumX4 - sumX3 * sumX3)
           - sumX  * (sumXY * sumX4  - sumX3 * sumX2Y)
           + sumX2 * (sumXY * sumX3  - sumX2Y* sumX2)) / D;
  float b = (n     * (sumXY * sumX4  - sumX3 * sumX2Y)
           - sumY  * (sumX  * sumX4  - sumX2 * sumX3)
           + sumX2 * (sumY  * sumX3  - sumX  * sumX2Y)) / D;
  float c = (n     * (sumX2 * sumX2Y - sumX3 * sumXY)
           - sumX  * (sumX  * sumX2Y - sumXY * sumX3)
           + sumY  * (sumX  * sumX3  - sumX2 * sumX2)) / D;

  float totalError = 0.0f;
  for (int i = 0; i < n; i++) {
    float est = a + b * x[i] + c * x[i] * x[i];
    totalError += fabsf(y[i] - est);
  }
  return totalError;
}

// ── Monte Carlo contaminant search ───────────────────────────────────────────
void monteCarloSimulation(float *empiricalTemps, float *empiricalDPs, int n) {
  if (n == 0) return;

  float bestError = 1e6f;
  float bestCO2   = 1.0f;
  float bestSO2   = 1.0f;
  float bestNO2   = 1.0f;

  // FIX 8a: Cache ambient humidity once before the loops.
  // The original code called bme.readHumidity() on every inner iteration —
  // with step 0.002 that means 100^3 = 1 000 000 I²C sensor reads, which is
  // completely impractical on an Arduino.
  float cachedHumidity = bme.readHumidity();

  // FIX 8b: Step size widened from 0.002 to 0.01 → 21 steps per dimension →
  // ~9 261 total iterations, feasible in a few seconds on an Arduino.
  // Reduce further to 0.02 (11 steps/dim, ~1 331 iter) if speed is critical.
  for (float co2Dev = 0.9f; co2Dev <= 1.101f; co2Dev += 0.01f) {
    for (float so2Dev = 0.9f; so2Dev <= 1.101f; so2Dev += 0.01f) {
      for (float no2Dev = 0.9f; no2Dev <= 1.101f; no2Dev += 0.01f) {

        // FIX 8c: Replaced the C99 VLA float simulatedDewPoints[n] with a
        // fixed-size array — VLAs are not guaranteed in Arduino C++.
        float simDP[totalDataPoints];
        for (int j = 0; j < n; j++) {
          float baseDP = calculateDewPoint(empiricalTemps[j], cachedHumidity);
          simDP[j] = adjustDewPointForContaminants(baseDP, co2Dev, so2Dev, no2Dev);
        }

        // FIX 9: Compare simulated DPs against *empirical* DPs via RMSE.
        // The original compared simulated data against itself (wrong metric).
        float err = computeRMSE(simDP, empiricalDPs, n);

        if (err < bestError) {
          bestError = err;
          bestCO2   = co2Dev;
          bestSO2   = so2Dev;
          bestNO2   = no2Dev;
        }
      }
    }
  }

  Serial.println("\n=== Monte Carlo Results ===");
  Serial.print("Best CO2 deviation: ");
  Serial.print((bestCO2 - 1.0f) * 100.0f, 2);
  Serial.println(" %");
  Serial.print("Best SO2 deviation: ");
  Serial.print((bestSO2 - 1.0f) * 100.0f, 2);
  Serial.println(" %");
  Serial.print("Best NO2 deviation: ");
  Serial.print((bestNO2 - 1.0f) * 100.0f, 2);
  Serial.println(" %");
  Serial.print("RMSE: ");
  Serial.print(bestError, 4);
  Serial.println(" °C");
}

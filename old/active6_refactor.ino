#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT4x.h>

// Sensor instances
Adafruit_BME280 bme;
Adafruit_SHT4x sht4x;

// Constants for Magnus-Tetens formula and sensor setup
const float A = 17.27;
const float B = 237.7;
const float pressureFactor = 0.001;

// Gas influence factors and non-linear scaling
const float gasFactors[3] = {0.003, 0.005, 0.004};    // CO2, SO2, NO2 factors
const float nonlinearFactors[3] = {1.5, 1.3, 1.4};    // Non-linear scaling factors for CO2, SO2, NO2

// PWM and heater control settings
const int coolerPin = 9;
const int heaterPin = 8;
const int maxPWM = 255;
const int minPWM = 50;
const int totalDataPoints = 19;

// Arrays for data storage
float empiricalDewPoints[totalDataPoints];
float empiricalTemperatures[totalDataPoints];
int dataPointIndex = 0;

// Function prototypes
float calculateDewPoint(float temperature, float humidity);
float adjustDewPointForPressure(float dewPoint, float pressure);
float adjustDewPointForGasNonlinear(float dewPoint, float concentration, float gasFactor, float nonlinearFactor);
float adjustDewPointForContaminants(float dewPoint, float co2Deviation, float so2Deviation, float no2Deviation);
void controlCoolingPWM(float targetTemperature);
void createCoolingProfile(float estimatedDewPoint);
void verifyDewPointWithHeatingProfile();
float fitPolynomialCurve(float *x, float *y, int n);
void monteCarloSimulation(float *empiricalTemperatures, float *empiricalDewPoints, int n);

void setup() {
  Serial.begin(9600);
  initializeSensors();
  float estimatedDewPoint = estimateInitialDewPoint();
  createCoolingProfile(estimatedDewPoint);
  verifyDewPointWithHeatingProfile();
  monteCarloSimulation(empiricalTemperatures, empiricalDewPoints, dataPointIndex);
}

void loop() {
  delay(5000); // Prevent continuous execution of the main loop
}

// Initialize sensors and check for proper connection
void initializeSensors() {
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  if (!sht4x.begin()) {
    Serial.println("Could not find a valid SHT4x sensor, check wiring!");
    while (1);
  }

  pinMode(coolerPin, OUTPUT);
  analogWrite(coolerPin, 0); // Ensure cooling is off initially
  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW); // Heater off initially
}

// Estimate the initial dew point from ambient conditions
float estimateInitialDewPoint() {
  float initialTemperature = bme.readTemperature();
  float initialHumidity = bme.readHumidity();
  return calculateDewPoint(initialTemperature, initialHumidity);
}

// Calculate dew point using Magnus-Tetens formula
float calculateDewPoint(float temperature, float humidity) {
  float alpha = ((A * temperature) / (B + temperature)) + log(humidity / 100.0);
  return (B * alpha) / (A - alpha);
}

// Adjust dew point based on barometric pressure
float adjustDewPointForPressure(float dewPoint, float pressure) {
  return dewPoint + (pressure - 1013.25) * pressureFactor;
}

// Apply nonlinear gas effects on dew point
float adjustDewPointForGasNonlinear(float dewPoint, float concentration, float gasFactor, float nonlinearFactor) {
  float adjustment = gasFactor * pow(concentration, nonlinearFactor);
  return dewPoint + dewPoint * adjustment;
}

// Adjust dew point for combined contaminant gas effects
float adjustDewPointForContaminants(float dewPoint, float co2Deviation, float so2Deviation, float no2Deviation) {
  dewPoint = adjustDewPointForGasNonlinear(dewPoint, co2Deviation, gasFactors[0], nonlinearFactors[0]);
  dewPoint = adjustDewPointForGasNonlinear(dewPoint, so2Deviation, gasFactors[1], nonlinearFactors[1]);
  return adjustDewPointForGasNonlinear(dewPoint, no2Deviation, gasFactors[2], nonlinearFactors[2]);
}

// Adaptive cooling control with feedback
void controlCoolingPWM(float targetTemperature) {
  int pwmValue = maxPWM;
  analogWrite(coolerPin, pwmValue);

  while (true) {
    float currentTemperature = bme.readTemperature();
    if (abs(currentTemperature - targetTemperature) < 0.1) {
      break;
    }
    pwmValue = map(currentTemperature, targetTemperature - 5, targetTemperature + 5, maxPWM, minPWM);
    analogWrite(coolerPin, constrain(pwmValue, minPWM, maxPWM));
    delay(200);
  }
}

// Create a cooling profile to collect dew point data points
void createCoolingProfile(float estimatedDewPoint) {
  float targetTemperatures[totalDataPoints];

  // Define temperature steps around the estimated dew point
  for (int i = 0; i < 3; i++) {
    targetTemperatures[i] = estimatedDewPoint + (i == 0 ? -5.0 : 5.0);
  }
  for (int i = 3; i < totalDataPoints; i++) {
    targetTemperatures[i] = estimatedDewPoint - 1.0 + ((i - 3) * 0.125);
  }

  // Data collection loop with adaptive cooling
  for (int i = 0; i < totalDataPoints; i++) {
    controlCoolingPWM(targetTemperatures[i]);
    delay(1000); // Allow stabilization time

    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;

    float dewPoint = calculateDewPoint(temperature, humidity);
    dewPoint = adjustDewPointForPressure(dewPoint, pressure);

    empiricalTemperatures[dataPointIndex] = temperature;
    empiricalDewPoints[dataPointIndex] = dewPoint;
    dataPointIndex++;

    Serial.print("Cooling Phase - Temperature: ");
    Serial.print(temperature);
    Serial.print(" °C, Humidity: ");
    Serial.print(humidity);
    Serial.print(" %, Dew Point: ");
    Serial.print(dewPoint);
    Serial.println(" °C");
  }

  controlCoolingPWM(0); // Turn off cooling
}

// Adaptive heater control for non-linear condensation effects
void verifyDewPointWithHeatingProfile() {
  const int heatingDataPoints = 10;
  float heatingTemperatures[heatingDataPoints];
  float heatingHumidity[heatingDataPoints];
  float heatingDewPoints[heatingDataPoints];

  sht4x.setHeater(SHT4X_HEATER_MEDIUM, 5); // Shorter heating duration to minimize impact
  delay(50);

  for (int i = 0; i < heatingDataPoints; i++) {
    sensors_event_t humidityEvent, tempEvent;
    sht4x.getEvent(&humidityEvent, &tempEvent);

    heatingTemperatures[i] = tempEvent.temperature;
    heatingHumidity[i] = humidityEvent.relative_humidity;
    heatingDewPoints[i] = calculateDewPoint(tempEvent.temperature, humidityEvent.relative_humidity);

    delay(100);
  }

  float nonLinearEffect = 0;
  for (int i = 0; i < heatingDataPoints; i++) {
    Serial.print("Heating Temp: ");
    Serial.print(heatingTemperatures[i]);
    Serial.print(" °C, Humidity: ");
    Serial.print(heatingHumidity[i]);
    Serial.print(" %, Dew Point: ");
    Serial.print(heatingDewPoints[i]);
    Serial.println(" °C");

    nonLinearEffect += abs(heatingDewPoints[i] - empiricalDewPoints[totalDataPoints - 1]) / heatingDataPoints;
  }

  Serial.print("Non-linear condensation effect magnitude: ");
  Serial.println(nonLinearEffect);

  sht4x.setHeater(SHT4X_HEATER_OFF, 0);
}

// Polynomial fitting function with error analysis
float fitPolynomialCurve(float *x, float *y, int n) {
  float sumX = 0, sumY = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
  float sumXY = 0, sumX2Y = 0;

  for (int i = 0; i < n; i++) {
    sumX += x[i];
    sumY += y[i];
    sumX2 += x[i] * x[i];
    sumX3 += x[i] * x[i] * x[i];
    sumX4 += x[i] * x[i] * x[i] * x[i];
    sumXY += x[i] * y[i];
    sumX2Y += x[i] * x[i] * y[i];
}

  float denominator = n * (sumX2 * sumX4 - sumX3 * sumX3) - sumX * (sumX * sumX4 - sumX2 * sumX3) + sumX2 * (sumX * sumX3 - sumX2 * sumX2);
  if (denominator == 0) return 1e6;  // Prevent division by zero in the fitting calculation

  // Calculate polynomial coefficients
  float a = (sumY * (sumX2 * sumX4 - sumX3 * sumX3) - sumX * (sumXY * sumX4 - sumX3 * sumX2Y) + sumX2 * (sumXY * sumX3 - sumX2Y * sumX2)) / denominator;
  float b = (n * (sumXY * sumX4 - sumX3 * sumX2Y) - sumY * (sumX * sumX4 - sumX2 * sumX3) + sumX2 * (sumY * sumX3 - sumX * sumX2Y)) / denominator;
  float c = (n * (sumX2 * sumX2Y - sumX3 * sumXY) - sumX * (sumX * sumX2Y - sumXY * sumX3) + sumY * (sumX * sumX3 - sumX2 * sumX2)) / denominator;

  float totalError = 0;
  for (int i = 0; i < n; i++) {
    float estimatedY = a + b * x[i] + c * x[i] * x[i];
    totalError += abs(y[i] - estimatedY);
  }

  // Return the total error as a measure of the curve's accuracy
  return totalError;
}

// Monte Carlo simulation function to match empirical data with hypothetical contaminant curves
void monteCarloSimulation(float *empiricalTemperatures, float *empiricalDewPoints, int n) {
  float bestMatchError = 1e6;  // Initialize with a very high error value
  float bestCO2 = 1.0;         // Initial best guess for CO2 deviation
  float bestSO2 = 1.0;         // Initial best guess for SO2 deviation
  float bestNO2 = 1.0;         // Initial best guess for NO2 deviation

  // Iterate over possible deviations of gas concentrations to find the best match
  for (float co2Deviation = 0.9; co2Deviation <= 1.1; co2Deviation += 0.002) {
    for (float so2Deviation = 0.9; so2Deviation <= 1.1; so2Deviation += 0.002) {
      for (float no2Deviation = 0.9; no2Deviation <= 1.1; no2Deviation += 0.002) {
        float simulatedDewPoints[n];
        for (int j = 0; j < n; j++) {
          float baseDewPoint = calculateDewPoint(empiricalTemperatures[j], bme.readHumidity());
          simulatedDewPoints[j] = adjustDewPointForContaminants(baseDewPoint, co2Deviation, so2Deviation, no2Deviation);
        }

        float matchError = fitPolynomialCurve(empiricalTemperatures, simulatedDewPoints, n);

        // Check if the current simulation has the lowest error
        if (matchError < bestMatchError) {
          bestMatchError = matchError;
          bestCO2 = co2Deviation;
          bestSO2 = so2Deviation;
          bestNO2 = no2Deviation;
        }
      }
    }
  }

  // Output the best matching deviations found
  Serial.print("Best Matching CO2 Deviation: ");
  Serial.print((bestCO2 - 1.0) * 100);
  Serial.println(" %");
  Serial.print("Best Matching SO2 Deviation: ");
  Serial.print((bestSO2 - 1.0) * 100);
  Serial.println(" %");
  Serial.print("Best Matching NO2 Deviation: ");
  Serial.print((bestNO2 - 1.0) * 100);
  Serial.println(" %");
  Serial.print("Matching Error: ");
  Serial.println(bestMatchError);
}

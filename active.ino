#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Create an instance of the BME280 sensor
Adafruit_BME280 bme;

// Constants for the Magnus-Tetens formula
const float A = 17.27;
const float B = 237.7;

// Constants for barometric pressure influence
const float pressureFactor = 0.001; // Effect factor of pressure on dew point

// Empirical coefficients and scaling factors for contaminant gases
const float CO2Factor = 0.003;   // Base influence factor for CO2
const float SO2Factor = 0.005;   // Base influence factor for SO2
const float NO2Factor = 0.004;   // Base influence factor for NO2

// Nonlinear scaling factors
const float CO2NonlinearFactor = 1.5;  // Exponential scaling factor for CO2
const float SO2NonlinearFactor = 1.3;  // Exponential scaling factor for SO2
const float NO2NonlinearFactor = 1.4;  // Exponential scaling factor for NO2

// Array to store empirical dew point data points
const int maxDataPoints = 10;      // Number of data points to collect
float empiricalDewPoints[maxDataPoints];
float empiricalTemperatures[maxDataPoints];
int dataPointIndex = 0;

// Function to calculate dew point using the Magnus-Tetens formula
float calculateDewPoint(float temperature, float humidity) {
  float alpha = ((A * temperature) / (B + temperature)) + log(humidity / 100.0);
  float dewPoint = (B * alpha) / (A - alpha);
  return dewPoint;
}

// Function to adjust dew point based on barometric pressure
float adjustDewPointForPressure(float dewPoint, float pressure) {
  // Adjust dew point based on pressure
  float adjustedDewPoint = dewPoint + (pressure - 1013.25) * pressureFactor;
  return adjustedDewPoint;
}

// Function to adjust dew point for a specific gas using nonlinear effects
float adjustDewPointForGasNonlinear(float dewPoint, float concentration, float gasFactor, float nonlinearFactor) {
  // Apply a nonlinear adjustment based on gas concentration and nonlinear scaling factor
  float adjustment = gasFactor * pow(concentration, nonlinearFactor);
  return dewPoint + dewPoint * adjustment;
}

// Function to adjust dew point for combined nonlinear gas effects
float adjustDewPointForContaminantsNonlinear(float dewPoint, float co2Deviation, float so2Deviation, float no2Deviation) {
  // Adjust for each gas individually with nonlinear effects
  float dewPointCO2 = adjustDewPointForGasNonlinear(dewPoint, co2Deviation, CO2Factor, CO2NonlinearFactor);
  float dewPointSO2 = adjustDewPointForGasNonlinear(dewPointCO2, so2Deviation, SO2Factor, SO2NonlinearFactor);
  float dewPointNO2 = adjustDewPointForGasNonlinear(dewPointSO2, no2Deviation, NO2Factor, NO2NonlinearFactor);
  
  return dewPointNO2;
}

// Function to control thermoelectric cooling and record dew point data
void controlCoolingAndRecordData() {
  // Control the thermoelectric cooling device to gradually cool the sensor
  // Assuming digital pin 9 controls the cooling device
  pinMode(9, OUTPUT);

  for (int i = 0; i < maxDataPoints; i++) {
    digitalWrite(9, HIGH); // Turn on the cooling device
    delay(2000);           // Wait for the sensor to cool

    // Read sensor values
    float temperature = bme.readTemperature();     // in °C
    float humidity = bme.readHumidity();           // in %

    // Calculate the dew point at the cooled condition
    float dewPoint = calculateDewPoint(temperature, humidity);

    // Record the data
    empiricalTemperatures[dataPointIndex] = temperature;
    empiricalDewPoints[dataPointIndex] = dewPoint;
    dataPointIndex++;

    digitalWrite(9, LOW); // Turn off the cooling device
    delay(2000);          // Wait before next cooling phase
  }
}

// Function to fit a polynomial curve to the empirical data
float fitPolynomialCurve(float *x, float *y, int n) {
  // Placeholder function - in practice, use a curve fitting algorithm
  // Here, we assume we derive polynomial coefficients that describe the curve
  // A more precise method could use external tools or libraries for polynomial fitting.
  // This function should return a measure of the fit quality.
  float sumError = 0;
  for (int i = 0; i < n; i++) {
    float estimatedY = x[i] * x[i] * 0.001 - x[i] * 0.1 + 10; // Sample fitting function
    sumError += abs(y[i] - estimatedY); // Calculate error
  }
  return sumError;
}

// Monte Carlo simulation to find matching hypothetical curves
void monteCarloSimulation(float *empiricalTemperatures, float *empiricalDewPoints, int n) {
  float bestMatchError = 1000000; // Initialize with a large error value
  float bestCO2 = 0, bestSO2 = 0, bestNO2 = 0;

  // Run Monte Carlo simulations with random contaminant proportions
  for (int i = 0; i < 1000; i++) {
    float co2Deviation = random(-100, 100) / 1000.0; // Random deviation from -0.1 to 0.1
    float so2Deviation = random(-100, 100) / 1000.0;
    float no2Deviation = random(-100, 100) / 1000.0;

    // Calculate hypothetical dew points based on Monte Carlo variations
    float simulatedDewPoints[maxDataPoints];
    for (int j = 0; j < n; j++) {
      float baseDewPoint = calculateDewPoint(empiricalTemperatures[j], bme.readHumidity());
      simulatedDewPoints[j] = adjustDewPointForContaminantsNonlinear(baseDewPoint, co2Deviation, so2Deviation, no2Deviation);
    }

    // Compare the hypothetical curve to the empirical curve
    float matchError = fitPolynomialCurve(empiricalTemperatures, simulatedDewPoints, n);

    // Update the best match
    if (matchError < bestMatchError) {
      bestMatchError = matchError;
      bestCO2 = co2Deviation;
      bestSO2 = so2Deviation;
      bestNO2 = no2Deviation;
    }
  }

  // Print the closest matching gas deviations
  Serial.print("Best Matching CO2 Deviation: ");
  Serial.println(bestCO2 * 100);
  Serial.print("Best Matching SO2 Deviation: ");
  Serial.println(bestSO2 * 100);
  Serial.print("Best Matching NO2 Deviation: ");
  Serial.println(bestNO2 * 100);
}

void setup() {
  Serial.begin(9600);
  // Initialize BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Initialize cooling control pin
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW); // Ensure cooling is off initially

  // Control cooling and record data points
  controlCoolingAndRecordData();

  // Run Monte Carlo simulation to match empirical data
  monteCarloSimulation(empiricalTemperatures, empiricalDewPoints, dataPointIndex);
}

void loop() {
  // Main loop can remain empty as setup runs the main logic
  delay(5000); // Pause to prevent continuous execution
}

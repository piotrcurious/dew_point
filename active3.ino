#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT4x.h>

// Create instances of the BME280 and SHT4x sensors
Adafruit_BME280 bme;
Adafruit_SHT4x sht4x;

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

// PWM settings for thermoelectric cooler control
const int coolerPin = 9;   // PWM pin controlling the cooling device
const int heaterPin = 8;   // Digital pin to control the SHT4x heater
const int maxPWM = 255;    // Maximum PWM value for full cooling power
const int minPWM = 50;     // Minimum PWM value for cooling control

// Array to store empirical dew point data points
const int totalDataPoints = 19;       // Total number of data points to collect (3 + 16)
float empiricalDewPoints[totalDataPoints];
float empiricalTemperatures[totalDataPoints];
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

// Function to control thermoelectric cooling device using PWM
void controlCoolingPWM(int pwmValue) {
  analogWrite(coolerPin, pwmValue);
}

// Function to collect data by creating a cooling profile
void createCoolingProfile(float estimatedDewPoint) {
  int pwmValue = maxPWM; // Start with maximum cooling power
  float targetTemperatures[totalDataPoints];
  
  // Generate cooling profile temperatures
  // First 3 points are 5°C away from the estimated dew point
  for (int i = 0; i < 3; i++) {
    targetTemperatures[i] = estimatedDewPoint + (i == 0 ? -5.0 : 5.0); // 2 points below and above the dew point
  }
  
  // Generate 16 points covering the dew point range
  for (int i = 3; i < totalDataPoints; i++) {
    targetTemperatures[i] = estimatedDewPoint - 1.0 + ((i - 3) * 0.125); // Fine coverage around the dew point
  }

  // Control cooling and collect data points
  for (int i = 0; i < totalDataPoints; i++) {
    pwmValue = map(i, 0, totalDataPoints - 1, maxPWM, minPWM); // Gradually reduce PWM to target temperature
    controlCoolingPWM(pwmValue);
    delay(2000); // Allow the sensor to stabilize

    // Read sensor values
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float dewPoint = calculateDewPoint(temperature, humidity);

    // Store empirical data points
    empiricalTemperatures[dataPointIndex] = temperature;
    empiricalDewPoints[dataPointIndex] = dewPoint;
    dataPointIndex++;
  }

  // Turn off cooling after data collection
  controlCoolingPWM(0);
}

// Function to verify dew point by collecting heating profile with SHT4x sensor heater
void verifyDewPointWithHeatingProfile() {
  sht4x.begin();

  const int heatingDataPoints = 10; // Number of data points during heating
  float heatingTemperatures[heatingDataPoints];
  float heatingHumidity[heatingDataPoints];
  float heatingDewPoints[heatingDataPoints];

  // Activate the SHT4x heater briefly to evaporate condensation
  sht4x.setHeater(SHT4X_HEATER_MEDIUM, 100); // Medium heating for 100 milliseconds
  delay(200); // Allow some time to observe immediate response

  // Collect data during the heating process
  for (int i = 0; i < heatingDataPoints; i++) {
    sensors_event_t humidityEvent, tempEvent;
    sht4x.getEvent(&humidityEvent, &tempEvent);

    // Store temperature, humidity, and calculated dew point
    heatingTemperatures[i] = tempEvent.temperature;
    heatingHumidity[i] = humidityEvent.relative_humidity;
    heatingDewPoints[i] = calculateDewPoint(tempEvent.temperature, humidityEvent.relative_humidity);

    delay(100); // Small delay to collect time-resolved data
  }

  // Analyze collected heating data for non-linear condensation effects
  for (int i = 0; i < heatingDataPoints; i++) {
    Serial.print("Heating Temp: ");
    Serial.print(heatingTemperatures[i]);
    Serial.print(" °C, Humidity: ");
    Serial.print(heatingHumidity[i]);
    Serial.print(" %, Dew Point: ");
    Serial.print(heatingDewPoints[i]);
    Serial.println(" °C");
  }

  // Compare heating profile against initial cooling profile to identify non-linear condensation effects
  float nonLinearEffect = 0;
  for (int i = 0; i < heatingDataPoints; i++) {
    nonLinearEffect += abs(heatingDewPoints[i] - empiricalDewPoints[totalDataPoints - 1]) / heatingDataPoints;
  }

  Serial.print("Non-linear condensation effect magnitude: ");
  Serial.println(nonLinearEffect);

  // Assess if the observed non-linear effect is within acceptable bounds
  if (nonLinearEffect < 0.5) {
    Serial.println("Dew point verification successful: Non-linear effects minimal.");
  } else {
    Serial.println("Dew point verification indicates significant non-linear effects due to condensation.");
  }
}

// Function to fit a polynomial curve to the empirical data
float fitPolynomialCurve(float *x, float *y, int n) {
  // Placeholder function - in practice, use a curve fitting algorithm
  float sumError = 0;
  for (int i = 0; i < n; i++) {
    float estimatedY = x[i] * x[i] * 0.001 - x[i] * 0.1 + 10; // Sample fitting function
    sumError += abs(y[i] - estimatedY); // Calculate error
  }
  return sumError;
}

// Monte Carlo simulation to find matching hypothetical curves
// Initialize with a high value to find the minimum error
  float bestMatchError = 1e6;
  float bestCO2 = 0;
  float bestSO2 = 0;
  float bestNO2 = 0;

  // Run Monte Carlo simulations with varying contaminant gas deviations
  for (float co2Deviation = 0.8; co2Deviation <= 1.2; co2Deviation += 0.01) {
    for (float so2Deviation = 0.8; so2Deviation <= 1.2; so2Deviation += 0.01) {
      for (float no2Deviation = 0.8; no2Deviation <= 1.2; no2Deviation += 0.01) {
        
        // Calculate hypothetical dew points based on Monte Carlo variations
        float simulatedDewPoints[totalDataPoints];
        for (int j = 0; j < n; j++) {
          float baseDewPoint = calculateDewPoint(empiricalTemperatures[j], bme.readHumidity());
          simulatedDewPoints[j] = adjustDewPointForContaminantsNonlinear(baseDewPoint, co2Deviation, so2Deviation, no2Deviation);
        }

        // Compare the hypothetical curve to the empirical curve
        float matchError = fitPolynomialCurve(empiricalTemperatures, simulatedDewPoints, n);

        // Update the best match if the current error is smaller
        if (matchError < bestMatchError) {
          bestMatchError = matchError;
          bestCO2 = co2Deviation;
          bestSO2 = so2Deviation;
          bestNO2 = no2Deviation;
        }
      }
    }
  }

  // Print the closest matching gas deviations
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

void setup() {
  Serial.begin(9600);

  // Initialize BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Initialize SHT4x sensor
  if (!sht4x.begin()) {
    Serial.println("Could not find a valid SHT4x sensor, check wiring!");
    while (1);
  }

  // Initialize PWM control pin for cooling
  pinMode(coolerPin, OUTPUT);
  analogWrite(coolerPin, 0); // Ensure cooling is off initially

  // Initialize the heater control pin for SHT4x
  pinMode(heaterPin, OUTPUT);
  digitalWrite(heaterPin, LOW); // Heater off initially

  // Estimate initial dew point based on ambient conditions
  float initialTemperature = bme.readTemperature();
  float initialHumidity = bme.readHumidity();
  float estimatedDewPoint = calculateDewPoint(initialTemperature, initialHumidity);

  // Create cooling profile and collect data
  createCoolingProfile(estimatedDewPoint);

  // Verify the dew point using SHT4x sensor with heating profile analysis
  verifyDewPointWithHeatingProfile();

  // Run Monte Carlo simulation to match empirical data with hypothetical contaminant curves
  monteCarloSimulation(empiricalTemperatures, empiricalDewPoints, dataPointIndex);
}

void loop() {
  // Main loop can remain empty as setup runs the main logic
  delay(5000); // Pause to prevent continuous execution
}

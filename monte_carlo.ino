#include <Wire.h>             // For I2C communication
#include <Adafruit_Sensor.h>  // For sensors
#include <Adafruit_BME280.h>  // For BME280 sensor

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

// Accessory function to verify dew point with ±10% contamination gas deviations and nonlinear effects
void verifyDewPointNonlinear(float dewPoint) {
  float dewPointPlus10 = adjustDewPointForContaminantsNonlinear(dewPoint, 0.10, 0.10, 0.10);
  float dewPointMinus10 = adjustDewPointForContaminantsNonlinear(dewPoint, -0.10, -0.10, -0.10);

  Serial.print("Dew Point with +10% Nonlinear Contamination Gas Effect: ");
  Serial.print(dewPointPlus10);
  Serial.println(" °C");

  Serial.print("Dew Point with -10% Nonlinear Contamination Gas Effect: ");
  Serial.print(dewPointMinus10);
  Serial.println(" °C");
}

void setup() {
  Serial.begin(9600);
  // Initialize BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Read sensor values
  float temperature = bme.readTemperature();     // in °C
  float humidity = bme.readHumidity();           // in %
  float pressure = bme.readPressure() / 100.0F;  // in hPa

  // Calculate the basic dew point
  float dewPoint = calculateDewPoint(temperature, humidity);

  // Adjust the dew point for barometric pressure
  float dewPointAdjusted = adjustDewPointForPressure(dewPoint, pressure);

  // Calculate combined nonlinear contaminant effects assuming 0% deviation initially
  float dewPointFinal = adjustDewPointForContaminantsNonlinear(dewPointAdjusted, 0.0, 0.0, 0.0);

  // Display the main dew point calculation
  Serial.print("Calculated Dew Point (Adjusted for Nonlinear Contaminants): ");
  Serial.print(dewPointFinal);
  Serial.println(" °C");

  // Accessory function for ±10% contamination gas deviations with nonlinear effects
  verifyDewPointNonlinear(dewPointFinal);

  delay(2000); // Wait before the next reading
}

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT4x.h>
#include <math.h>
#include <string.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// ── Persistent Storage ───────────────────────────────────────────────────────
Preferences preferences;

// ── Sensor instances ─────────────────────────────────────────────────────────
Adafruit_BME280 bme;
Adafruit_SHT4x  sht4x;

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

// ── PWM / pin settings ───────────────────────────────────────────────────────
// Pins 16 and 17 are safe on most ESP32 boards (avoid 6-11).
const int   coolerPin         = 16;
const int   heaterPin         = 17;        // Reserved for external heater
const int   maxPWM            = 255;
const int   minPWM            = 50;
const int   numCoolingPoints  = 100;
const int   numHeatingPoints  = 50;
const int   totalDataPoints   = 500; // Large buffer for RT monitoring

// FIX 3: Safety timeout for the cooling control loop (5 minutes).
const unsigned long coolingTimeoutMs = 300000UL;

// ── Filtering Constants ──────────────────────────────────────────────────────
const float emaAlphaT = 0.2f;  // Temperature smoothing
const float emaAlphaH = 0.1f;  // Humidity smoothing (slower)
const float emaAlphaP = 0.5f;  // Pressure smoothing

// ── Filtered Values ──────────────────────────────────────────────────────────
float filteredT = 25.0f;
float filteredH = 50.0f;
float filteredP = 1013.25f;

// ── Global factors (refined over time) ───────────────────────────────────────
float currentCO2Factor = 1.0f;
float currentSO2Factor = 1.0f;
float currentNO2Factor = 1.0f;
float currentConfidence = 0.0f;
float coolingHealth    = 1.0f;
float ambientRefTemp   = 25.0f;
SemaphoreHandle_t factorMutex;

// ── Empirical data storage (Circular buffer for real-time monitoring) ────────
float empiricalDewPoints[totalDataPoints];
float empiricalTemperatures[totalDataPoints];
float empiricalHumidities[totalDataPoints];
float empiricalPressures[totalDataPoints];
int   dataPointIndex = 0;
bool  bufferFull = false;
SemaphoreHandle_t dataMutex;

// ── Task local static buffers (moved to global for setup access) ─────────────
float copyTemps[totalDataPoints], copyHums[totalDataPoints], copyPress[totalDataPoints];
float rawDewPoints[totalDataPoints];    // Optimized: Pre-calculate to speed up MC passes
float adjDP_global[totalDataPoints];    // Shared by MC search passes
float weights_global[totalDataPoints];  // Shared by MC search passes
float vTemps_global[totalDataPoints];   // Corresponding temps for correlation

// ── Web Server & WebSockets ──────────────────────────────────────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char* htmlContent = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Dew Point Monitor</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <style>
        body { margin: 0; overflow: hidden; background: #111; color: #eee; font-family: sans-serif; }
        #info { position: absolute; top: 10px; left: 10px; z-index: 10; }
        #controls { position: absolute; top: 10px; right: 10px; background: rgba(0,0,0,0.5); padding: 10px; border-radius: 5px; z-index: 10; }
        input { width: 50px; background: #333; color: #fff; border: 1px solid #555; }
        button { cursor: pointer; background: #00aaff; color: #fff; border: none; padding: 5px 10px; border-radius: 3px; }
    </style>
</head>
<body>
    <div id="info">
        <h1>Dew Point Monitor</h1>
        <div id="stats">Connecting...</div>
    </div>
    <div id="controls">
        <h3>Calibration Overrides</h3>
        CO2: <input type="number" id="ico2" step="0.1"><br>
        SO2: <input type="number" id="iso2" step="0.1"><br>
        NO2: <input type="number" id="ino2" step="0.1"><br>
        <button onclick="updateCal()">Update</button>
    </div>
    <script>
        let scene = new THREE.Scene();
        let camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        let renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(window.innerWidth, window.innerHeight);
        document.body.appendChild(renderer.domElement);

        let geometry = new THREE.BufferGeometry();
        let pointsCount = 500;
        let positions = new Float32Array(pointsCount * 3);
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        let material = new THREE.PointsMaterial({ color: 0x00aaff, size: 0.5 });
        let points = new THREE.Points(geometry, material);
        scene.add(points);

        camera.position.z = 50;

        let gateway = `ws://${window.location.hostname}/ws`;
        let websocket;
        function initWebSocket() {
            websocket = new WebSocket(gateway);
            websocket.onmessage = onMessage;
        }
        function onMessage(event) {
            let data = JSON.parse(event.data);
            document.getElementById('stats').innerHTML =
                `T: ${data.t.toFixed(2)} &deg;C | RH: ${data.h.toFixed(1)} %<br>` +
                `DP: ${data.dp.toFixed(2)} &deg;C | CorrDP: ${data.cdp.toFixed(2)} &deg;C<br>` +
                `CO2: ${data.co2.toFixed(3)} | SO2: ${data.so2.toFixed(3)} | NO2: ${data.no2.toFixed(3)}<br>` +
                `Fit Confidence: ${(data.conf*100).toFixed(1)}% | Cooling Health: ${(data.health*100).toFixed(1)}%`;

            document.getElementById('ico2').placeholder = data.co2.toFixed(2);
            document.getElementById('iso2').placeholder = data.so2.toFixed(2);
            document.getElementById('ino2').placeholder = data.no2.toFixed(2);

            // Update WebGL visualization (scroll points)
            for (let i = 0; i < pointsCount - 1; i++) {
                positions[i * 3 + 1] = positions[(i + 1) * 3 + 1];
                positions[i * 3 + 0] = (i / 10.0) - 25.0;
            }
            positions[(pointsCount - 1) * 3 + 1] = data.cdp - 15; // Offset for view
            positions[(pointsCount - 1) * 3 + 0] = ((pointsCount - 1) / 10.0) - 25.0;
            geometry.attributes.position.needsUpdate = true;
        }
        window.onload = initWebSocket;

        function updateCal() {
            let msg = {
                co2: parseFloat(document.getElementById('ico2').value),
                so2: parseFloat(document.getElementById('iso2').value),
                no2: parseFloat(document.getElementById('ino2').value)
            };
            websocket.send(JSON.stringify(msg));
        }

        function animate() {
            requestAnimationFrame(animate);
            renderer.render(scene, camera);
        }
        animate();
    </script>
</body>
</html>
)rawliteral";

// ── Function prototypes ───────────────────────────────────────────────────────
// FIX 6: Added missing prototypes for initializeSensors() and
//        estimateInitialDewPoint(), which are called before their definitions.
// FIX 7: Removed the orphaned adjustDewPointForGasNonlinear() prototype
//        (declared but never defined — replaced by the three per-gas functions).
void  initializeSensors();
float estimateInitialDewPoint();
float calculateDewPoint(float temperature, float humidity);
float adjustDewPointForPressure(float dewPoint, float pressure);
float getTempAdsorptionFactor(float temperature, float coeff);
float adjustDewPointForCO2(float dewPoint, float concentration, float temperature);
float adjustDewPointForSO2(float dewPoint, float concentration, float temperature);
float adjustDewPointForNO2(float dewPoint, float concentration, float temperature);
float adjustDewPointForContaminants(float dewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature);
float mapFloat(float x, float inMin, float inMax, float outMin, float outMax);
void  stopCooling();
void  controlCoolingPWM(float targetTemperature);
void  createCoolingProfile(float estimatedDewPoint, int numPoints);
void  verifyDewPointWithHeatingProfile();
float computeRMSE(float *simulated, float *empirical, int n);
float fitPolynomialCurve(float *x, float *y, int n);
float computeWeightedVariance(float *data, float *weights, int n);
float computeTemperatureCorrelation(float *dewPoints, float *temps, float *weights, int n);
float removeContaminantEffect(float measuredDewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature);
void  monteCarloSimulation(float *empiricalTemps, float *empiricalHumidities, float *empiricalPressures, int n, int headIndex);
void  addRealTimeDataPoint(float t, float h, float p, float dp);
void  monteCarloTask(void *parameter);
void  loadCalibration();
void  saveCalibration(float co2, float so2, float no2);
bool  checkSensorHealth();
void  enterSafeMode(const char* reason);
void  performSelfTest();

// ── Web Handlers ─────────────────────────────────────────────────────────────
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      char *msg = (char*)malloc(len + 1);
      memcpy(msg, data, len);
      msg[len] = '\0';

      // Robust manual JSON parsing (handles different order, spacing, and quotes)
      float f1 = -1, f2 = -1, f3 = -1;
      auto parseField = [](const char* payload, const char* field) -> float {
        const char* p = strstr(payload, field);
        if (!p) return -1.0f;
        p += strlen(field);
        // Skip over key-value separator and any whitespace/quotes
        while (*p && (*p == '\"' || *p == ' ' || *p == ':')) p++;
        if (*p == '\0' || *p == ',' || *p == '}') return -1.0f; // Invalid/empty value
        return (float)atof(p);
      };

      f1 = parseField(msg, "\"co2\"");
      f2 = parseField(msg, "\"so2\"");
      f3 = parseField(msg, "\"no2\"");

      xSemaphoreTake(factorMutex, portMAX_DELAY);
      if (f1 >= 0) currentCO2Factor = f1;
      if (f2 >= 0) currentSO2Factor = f2;
      if (f3 >= 0) currentNO2Factor = f3;
      xSemaphoreGive(factorMutex);

      if (f1 >= 0 || f2 >= 0 || f3 >= 0) {
        saveCalibration(currentCO2Factor, currentSO2Factor, currentNO2Factor);
      }
      free(msg);
    }
  }
}

// ── setup() ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  initializeSensors();
  performSelfTest();
  loadCalibration();

  // WiFi Setup (using stubs in mock)
  WiFi.begin("SSID", "PASS");

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", htmlContent);
  });
  server.begin();

  dataMutex = xSemaphoreCreateMutex();
  factorMutex = xSemaphoreCreateMutex();

  Serial.println("Starting initial calibration profile...");
  ambientRefTemp = bme.readTemperature();
  float estimatedDewPoint = estimateInitialDewPoint();
  createCoolingProfile(estimatedDewPoint, numCoolingPoints);
  verifyDewPointWithHeatingProfile();

  if (dataPointIndex > 1) {
    // Populate rawDewPoints before initial simulation
    for (int i = 0; i < dataPointIndex; i++) {
        float dp = calculateDewPoint(empiricalTemperatures[i], empiricalHumidities[i]);
        rawDewPoints[i] = adjustDewPointForPressure(dp, empiricalPressures[i]);
    }
    monteCarloSimulation(empiricalTemperatures, empiricalHumidities, empiricalPressures, dataPointIndex, dataPointIndex);
    if (dataPointIndex >= totalDataPoints) bufferFull = true;
  }

  // Start the background refinement task on Core 1
  xTaskCreatePinnedToCore(monteCarloTask, "MCTask", 10000, NULL, 1, NULL, 1);
}

void loop() {
  if (!checkSensorHealth()) {
    enterSafeMode("Sensor Failure");
    delay(5000);
    return;
  }

  float rawT = bme.readTemperature();
  float rawH = bme.readHumidity();
  float rawP = bme.readPressure() / 100.0f;

  // Apply EMA Filtering
  filteredT = (emaAlphaT * rawT) + (1.0f - emaAlphaT) * filteredT;
  filteredH = (emaAlphaH * rawH) + (1.0f - emaAlphaH) * filteredH;
  filteredP = (emaAlphaP * rawP) + (1.0f - emaAlphaP) * filteredP;

  float t = filteredT;
  float h = filteredH;
  float p = filteredP;

  float dp = calculateDewPoint(t, h);
  dp = adjustDewPointForPressure(dp, p);

  addRealTimeDataPoint(t, h, p, dp);

  float cCO2, cSO2, cNO2;
  xSemaphoreTake(factorMutex, portMAX_DELAY);
  cCO2 = currentCO2Factor; cSO2 = currentSO2Factor; cNO2 = currentNO2Factor;
  xSemaphoreGive(factorMutex);

  float correctedDP = removeContaminantEffect(dp, cCO2, cSO2, cNO2, t);

  Serial.print("RT [DP="); Serial.print(dp, 2);
  Serial.print(" CorrDP="); Serial.print(correctedDP, 2);
  Serial.print(" CO2="); Serial.print(cCO2, 2);
  Serial.println("]");

  // Send data to WebSockets
  char json[256];
  snprintf(json, sizeof(json), "{\"dp\":%.2f,\"cdp\":%.2f,\"co2\":%.3f,\"so2\":%.3f,\"no2\":%.3f,\"t\":%.2f,\"h\":%.1f,\"conf\":%.3f,\"health\":%.3f}",
           dp, correctedDP, cCO2, cSO2, cNO2, t, h, currentConfidence, coolingHealth);
  ws.textAll(json);

  delay(2000);
}

void monteCarloTask(void *parameter) {
  while (true) {
    vTaskDelay(60000 / portTICK_PERIOD_MS);
    if (bufferFull) {
      Serial.println("Background refining starting...");
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      int head = dataPointIndex;
      memcpy(copyTemps, empiricalTemperatures, sizeof(copyTemps));
      memcpy(copyHums, empiricalHumidities, sizeof(copyHums));
      memcpy(copyPress, empiricalPressures, sizeof(copyPress));
      xSemaphoreGive(dataMutex);

      // Pre-calculate raw dew points once to speed up the MC nested loops
      for (int i = 0; i < totalDataPoints; i++) {
        float dp = calculateDewPoint(copyTemps[i], copyHums[i]);
        rawDewPoints[i] = adjustDewPointForPressure(dp, copyPress[i]);
      }

      monteCarloSimulation(copyTemps, copyHums, copyPress, totalDataPoints, head);
    }
  }
}

void addRealTimeDataPoint(float t, float h, float p, float dp) {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  empiricalTemperatures[dataPointIndex] = t;
  empiricalHumidities[dataPointIndex]   = h;
  empiricalPressures[dataPointIndex]    = p;
  empiricalDewPoints[dataPointIndex]    = dp;

  dataPointIndex++;
  if (dataPointIndex >= totalDataPoints) {
    dataPointIndex = 0;
    bufferFull = true;
  }
  xSemaphoreGive(dataMutex);
}

// ── Sensor initialisation ────────────────────────────────────────────────────
void initializeSensors() {
  bool bme_ok = bme.begin(0x76);
  bool sht_ok = sht4x.begin();

  if (!bme_ok || !sht_ok) {
    if (!bme_ok) Serial.println("BME280 init failed!");
    if (!sht_ok) Serial.println("SHT4x init failed!");
    enterSafeMode("I2C Init Failure");
    return;
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

// Helper to get temperature-dependent adsorption factor.
float getTempAdsorptionFactor(float temperature, float coeff) {
  float deltaT = ambientRefTemp - temperature;
  if (deltaT < 0) deltaT = 0;
  return 1.0f + coeff * deltaT;
}

// ── Per-gas non-linear adjustments ──────────────────────────────────────────
const float co2TempCoeff = 0.02f;
const float so2TempCoeff = 0.05f;
const float no2TempCoeff = 0.08f;

float adjustDewPointForCO2(float dewPoint, float concentration, float temperature) {
  float adj = co2Factor * logf(1.0f + co2NonlinearCoeff * concentration) * getTempAdsorptionFactor(temperature, co2TempCoeff);
  return dewPoint + dewPoint * adj;
}

float adjustDewPointForSO2(float dewPoint, float concentration, float temperature) {
  float adj = so2Factor * powf(concentration, 2.0f) * so2NonlinearCoeff * getTempAdsorptionFactor(temperature, so2TempCoeff);
  return dewPoint + dewPoint * adj;
}

float adjustDewPointForNO2(float dewPoint, float concentration, float temperature) {
  float adj = no2Factor * (expf(no2NonlinearCoeff * concentration) - 1.0f) * getTempAdsorptionFactor(temperature, no2TempCoeff);
  return dewPoint + dewPoint * adj;
}

// ── Combined contaminant adjustment ─────────────────────────────────────────
float adjustDewPointForContaminants(float dewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature) {
  dewPoint = adjustDewPointForCO2(dewPoint, co2Dev, temperature);
  dewPoint = adjustDewPointForSO2(dewPoint, so2Dev, temperature);
  return    adjustDewPointForNO2(dewPoint, no2Dev, temperature);
}

float removeContaminantEffect(float measuredDewPoint, float co2Dev, float so2Dev, float no2Dev, float temperature) {
  float adj_co2 = co2Factor * logf(1.0f + co2NonlinearCoeff * co2Dev) * getTempAdsorptionFactor(temperature, co2TempCoeff);
  float adj_so2 = so2Factor * powf(so2Dev, 2.0f) * so2NonlinearCoeff * getTempAdsorptionFactor(temperature, so2TempCoeff);
  float adj_no2 = no2Factor * (expf(no2NonlinearCoeff * no2Dev) - 1.0f) * getTempAdsorptionFactor(temperature, no2TempCoeff);
  return measuredDewPoint / (1.0f + adj_co2 + adj_so2 + adj_no2);
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

// ── PID Control constants ────────────────────────────────────────────────────
const float Kp = 40.0f;
const float Ki = 1.0f;
const float Kd = 20.0f;
const float Kf = 5.0f; // Feed-forward gain

// ── Adaptive cooling with feedback (PID Controller) ──────────────────────────
void controlCoolingPWM(float targetTemperature) {
  float integral = 0.0f;
  unsigned long startMs = millis();
  unsigned long lastMs = startMs;
  float currentTemp = bme.readTemperature();
  float lastError = currentTemp - targetTemperature;

  while (true) {
    unsigned long now = millis();
    if (now - startMs > coolingTimeoutMs) {
      Serial.println("WARNING: Cooling timeout — target temperature not reached.");
      break;
    }

    float dtReal = (float)(now - lastMs) / 1000.0f;
    if (dtReal < 0.001f) dtReal = 0.001f;
    lastMs = now;

    currentTemp = bme.readTemperature();
    float error = currentTemp - targetTemperature;

    if (fabsf(error) < 0.1f) break;

    // Proportional term
    float P = Kp * error;

    // Integral term (with anti-windup)
    if (fabsf(error) < 2.0f) {
      integral += error * dtReal;
    } else {
      integral = 0.0f;
    }
    float I = Ki * integral;

    // Derivative term
    float D = Kd * (error - lastError) / dtReal;
    lastError = error;

    // Feed-forward based on temperature difference (cooling against ambient)
    float FF = Kf * (ambientRefTemp - targetTemperature);

    int pwmValue = (int)(P + I + D + FF);

    // --- Safety Throttling ---
    // In a real system, we would read these via analog pins
    float mosfetTemp = ambientRefTemp; // Use start-of-run ambient as proxy
    float supplyV = 12.0f;    // Placeholder

    // Example: Throttle if MOSFET > 70C
    if (mosfetTemp > 70.0f) pwmValue /= 2;
    if (mosfetTemp > 90.0f) pwmValue = 0;

    // Example: Throttle if supply voltage drops below 10V
    if (supplyV < 10.0f) pwmValue = min(pwmValue, 100);

    // --- Cooling Performance Monitor ---
    // If PWM is high but temperature error isn't reducing, health drops
    if (pwmValue > 200 && error > 1.0f && (now - lastMs > 5000)) {
        coolingHealth *= 0.99f; // Gradual decay if struggling
    } else if (fabsf(error) < 0.2f) {
        coolingHealth = (coolingHealth * 0.99f) + 0.01f; // Recover health if stable
    }

    // Allow PWM to drop to 0 if we are already below target temperature
    analogWrite(coolerPin, constrain(pwmValue, 0, maxPWM));
    delay(200);
  }
}

// ── Cooling profile (empirical data collection) ──────────────────────────────
float targetTemperatures[totalDataPoints];

void createCoolingProfile(float estimatedDewPoint, int numPoints) {
  if (numPoints > totalDataPoints) numPoints = totalDataPoints;

  for (int i = 0; i < 3; i++) {
    targetTemperatures[i] = estimatedDewPoint + (i - 1) * 5.0f;
  }
  float stepSize = 2.0f / (numPoints - 3);
  for (int i = 3; i < numPoints; i++) {
    targetTemperatures[i] = estimatedDewPoint - 1.0f + (i - 3) * stepSize;
  }

  for (int i = 0; i < numPoints; i++) {
    controlCoolingPWM(targetTemperatures[i]);
    delay(1000);  // Stabilisation pause

    float temp     = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0f;

    float dp = calculateDewPoint(temp, humidity);
    dp = adjustDewPointForPressure(dp, pressure);

    empiricalTemperatures[dataPointIndex] = temp;
    empiricalHumidities[dataPointIndex]   = humidity;
    empiricalPressures[dataPointIndex]    = pressure;
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
float heatingDewPoints[totalDataPoints];

void verifyDewPointWithHeatingProfile() {
  const int heatingDataPoints = numHeatingPoints;

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

    if (dataPointIndex < totalDataPoints) {
      sensors_event_t pressEvent;
      bme.readPressure(); // Dummy read if needed or use a real one
      float p = bme.readPressure() / 100.0f;

      empiricalTemperatures[dataPointIndex] = t;
      empiricalHumidities[dataPointIndex]   = h;
      empiricalPressures[dataPointIndex]    = p;
      empiricalDewPoints[dataPointIndex]    = heatingDewPoints[i];
      dataPointIndex++;
    }

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

// Weighted variance to prioritize recent data points.
float computeWeightedVariance(float *data, float *weights, int n) {
  if (n <= 1) return 1e6f;
  float totalWeight = 0.0f;
  float weightedSum = 0.0f;
  for (int i = 0; i < n; i++) {
    weightedSum += data[i] * weights[i];
    totalWeight += weights[i];
  }
  if (totalWeight < 1e-9f) return 1e6f;
  float weightedMean = weightedSum / totalWeight;
  float weightedSumSq = 0.0f;
  for (int i = 0; i < n; i++) {
    float d = data[i] - weightedMean;
    weightedSumSq += weights[i] * d * d;
  }
  return weightedSumSq / totalWeight;
}

// Multidimensional cost helper: checks if the corrected dew point is still
// correlated with temperature (indicating cross-sensitivity is not removed).
float computeTemperatureCorrelation(float *dewPoints, float *temps, float *weights, int n) {
  if (n <= 1) return 0.0f;
  float totalW = 0.0f, sumW_T = 0.0f, sumW_DP = 0.0f;
  for (int i = 0; i < n; i++) {
    sumW_T  += temps[i] * weights[i];
    sumW_DP += dewPoints[i] * weights[i];
    totalW  += weights[i];
  }
  float meanT = sumW_T / totalW;
  float meanDP = sumW_DP / totalW;

  float numerator = 0.0f, denT = 0.0f, denDP = 0.0f;
  for (int i = 0; i < n; i++) {
    float dT = temps[i] - meanT;
    float dDP = dewPoints[i] - meanDP;
    numerator += weights[i] * dT * dDP;
    denT += weights[i] * dT * dT;
    denDP += weights[i] * dDP * dDP;
  }
  if (denT < 1e-6f || denDP < 1e-6f) return 0.0f;
  return fabsf(numerator / sqrtf(denT * denDP));
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

// ── Monte Carlo contaminant search (Three-pass, Multidimensional Cost) ───────
void monteCarloSimulation(float *empiricalTemps, float *empiricalHumidities, float *empiricalPressures, int n, int headIndex) {
  if (n == 0) return;

  float bestError = 1e6f;
  float bestCO2   = currentCO2Factor;
  float bestSO2   = currentSO2Factor;
  float bestNO2   = currentNO2Factor;

  // Pre-calculate chronological weights for the circular buffer
  float weights_full[totalDataPoints];
  for (int j = 0; j < n; j++) {
    int age = (headIndex - 1 - j + n) % n;
    // Chronological weighting is NOT enough to separate drift.
    // Instead, we use flat weighting for the cost, but could use it for recent bias.
    weights_full[j] = 1.0f;
  }

  // Pass 1: Coarse search
  for (float co2 = 0.0f; co2 <= 5.01f; co2 += 1.0f) {
    for (float so2 = 0.0f; so2 <= 5.01f; so2 += 1.0f) {
      for (float no2 = 0.0f; no2 <= 5.01f; no2 += 1.0f) {
        int vp = 0;
        for (int j = 0; j < n; j++) {
          if (empiricalHumidities[j] < 95.0f) {
            float dp = rawDewPoints[j];
            adjDP_global[vp] = removeContaminantEffect(dp, co2, so2, no2, empiricalTemps[j]);
            weights_global[vp] = weights_full[j];
            vTemps_global[vp]  = empiricalTemps[j];
            vp++;
          }
        }
        if (vp < 2) continue;
        float var = computeWeightedVariance(adjDP_global, weights_global, vp);
        float corr = computeTemperatureCorrelation(adjDP_global, vTemps_global, weights_global, vp);
        float err = (100.0f * corr) + var + 0.1f * (co2 + so2 + no2);
        if (err < bestError) {
          bestError = err; bestCO2 = co2; bestSO2 = so2; bestNO2 = no2;
        }
      }
    }
  }

  // Pass 2: Medium search
  float startCO2m = max(0.0f, bestCO2 - 0.5f);
  float startSO2m = max(0.0f, bestSO2 - 0.5f);
  float startNO2m = max(0.0f, bestNO2 - 0.5f);

  for (float co2 = startCO2m; co2 <= min(5.0f, startCO2m + 1.01f); co2 += 0.2f) {
    for (float so2 = startSO2m; so2 <= min(5.0f, startSO2m + 1.01f); so2 += 0.2f) {
      for (float no2 = startNO2m; no2 <= min(5.0f, startNO2m + 1.01f); no2 += 0.2f) {
        int vp = 0;
        for (int j = 0; j < n; j++) {
          if (empiricalHumidities[j] < 95.0f) {
            float dp = rawDewPoints[j];
            adjDP_global[vp] = removeContaminantEffect(dp, co2, so2, no2, empiricalTemps[j]);
            weights_global[vp] = weights_full[j];
            vTemps_global[vp]  = empiricalTemps[j];
            vp++;
          }
        }
        if (vp < 2) continue;
        float var = computeWeightedVariance(adjDP_global, weights_global, vp);
        float corr = computeTemperatureCorrelation(adjDP_global, vTemps_global, weights_global, vp);
        float err = (100.0f * corr) + var + 0.1f * (co2 + so2 + no2);
        if (err < bestError) {
          bestError = err; bestCO2 = co2; bestSO2 = so2; bestNO2 = no2;
        }
      }
    }
  }

  // Pass 3: Fine search
  float startCO2f = max(0.0f, bestCO2 - 0.1f);
  float startSO2f = max(0.0f, bestSO2 - 0.1f);
  float startNO2f = max(0.0f, bestNO2 - 0.1f);

  for (float co2 = startCO2f; co2 <= min(5.0f, startCO2f + 0.201f); co2 += 0.05f) {
    for (float so2 = startSO2f; so2 <= min(5.0f, startSO2f + 0.201f); so2 += 0.05f) {
      for (float no2 = startNO2f; no2 <= min(5.0f, startNO2f + 0.201f); no2 += 0.05f) {
        int vp = 0;
        for (int j = 0; j < n; j++) {
          if (empiricalHumidities[j] < 95.0f) {
            float dp = rawDewPoints[j];
            adjDP_global[vp] = removeContaminantEffect(dp, co2, so2, no2, empiricalTemps[j]);
            weights_global[vp] = weights_full[j];
            vTemps_global[vp]  = empiricalTemps[j];
            vp++;
          }
        }
        if (vp < 2) continue;
        float var = computeWeightedVariance(adjDP_global, weights_global, vp);
        float corr = computeTemperatureCorrelation(adjDP_global, vTemps_global, weights_global, vp);
        float err = (100.0f * corr) + var + 0.1f * (co2 + so2 + no2);
        if (err < bestError) {
          bestError = err; bestCO2 = co2; bestSO2 = so2; bestNO2 = no2;
        }
      }
    }
  }

  Serial.println("\n=== Monte Carlo Results ===");
  Serial.print("Best CO2 factor: "); Serial.println(bestCO2, 3);
  Serial.print("Best SO2 factor: "); Serial.println(bestSO2, 3);
  Serial.print("Best NO2 factor: "); Serial.println(bestNO2, 3);
  Serial.print("Final Cost: ");    Serial.print(bestError, 6);

  // --- Convergence Check & Confidence Calculation ---
  // Confidence is inversely related to the bestError.
  // Assuming a 'good' fit has error < 1.0 (arbitrary threshold for normalized cost)
  float confidence = 1.0f / (1.0f + bestError);

  // Only update if confidence is reasonable or better than before
  xSemaphoreTake(factorMutex, portMAX_DELAY);
  bool improved = (confidence > currentConfidence * 1.05f) || (currentConfidence < 0.1f);
  if (improved) {
    currentCO2Factor = bestCO2;
    currentSO2Factor = bestSO2;
    currentNO2Factor = bestNO2;
    currentConfidence = confidence;
    Serial.println(" [UPDATED]");
    saveCalibration(bestCO2, bestSO2, bestNO2);
  } else {
    Serial.println(" [REJECTED - No significant improvement]");
  }
  xSemaphoreGive(factorMutex);
}

// ── Persistent Calibration Helpers ──────────────────────────────────────────
void loadCalibration() {
  preferences.begin("calibration", true); // Read-only
  currentCO2Factor = preferences.getFloat("co2", 1.0f);
  currentSO2Factor = preferences.getFloat("so2", 1.0f);
  currentNO2Factor = preferences.getFloat("no2", 1.0f);
  preferences.end();
  Serial.println("Calibration loaded from NVS.");
}

void saveCalibration(float co2, float so2, float no2) {
  preferences.begin("calibration", false); // Read-write
  preferences.putFloat("co2", co2);
  preferences.putFloat("so2", so2);
  preferences.putFloat("no2", no2);
  preferences.end();
  Serial.println("Calibration saved to NVS.");
}

bool checkSensorHealth() {
  float t = bme.readTemperature();
  float h = bme.readHumidity();

  // Basic range validation
  if (t < -40.0f || t > 85.0f) return false;
  if (h < 0.0f || h > 100.0f) return false;

  // NaN check
  if (isnan(t) || isnan(h)) return false;

  return true;
}

void enterSafeMode(const char* reason) {
  stopCooling();
  digitalWrite(heaterPin, LOW);
  Serial.print("CRITICAL: Entering Safe Mode. Reason: ");
  Serial.println(reason);

  // Broadcast error to dashboard
  char json[128];
  snprintf(json, sizeof(json), "{\"error\":\"%s\"}", reason);
  ws.textAll(json);
}

void performSelfTest() {
  Serial.println("--- System Self-Test ---");
  bool pass = true;
  if (!checkSensorHealth()) {
    Serial.println("Sensor Health: FAIL");
    pass = false;
  } else {
    Serial.println("Sensor Health: PASS");
  }

  float p = bme.readPressure() / 100.0f;
  if (p < 800.0f || p > 1200.0f) {
    Serial.print("Pressure check: FAIL ("); Serial.print(p); Serial.println(" hPa)");
    pass = false;
  } else {
    Serial.println("Pressure check: PASS");
  }

  if (!pass) {
    enterSafeMode("Self-Test Failed");
  } else {
    Serial.println("Self-Test: SUCCESS");
  }
}

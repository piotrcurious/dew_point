#include "config.h"
#include "Physics.h"
#include "Sensors.h"
#include "ThermalControl.h"
#include "Optimization.h"
#include "Dashboard.h"
#include <Preferences.h>

// ── Globals ──────────────────────────────────────────────────────────────────
Preferences preferences;
float filteredT = 25.0f, filteredH = 50.0f, filteredP = 1013.25f;
float currentCO2Factor = 1.0f, currentSO2Factor = 1.0f, currentNO2Factor = 1.0f;
float currentConfidence = 0.0f, coolingHealth = 1.0f, ambientRefTemp = 25.0f;
int globalCurrentPWM = 0;
uint32_t minHeapSeen = 0xFFFFFFFF, lastCalTime = 0;
SemaphoreHandle_t dataMutex, factorMutex;

void saveCalibration(float co2, float so2, float no2) {
  preferences.begin("calibration", false);
  preferences.putFloat("co2", co2); preferences.putFloat("so2", so2); preferences.putFloat("no2", no2);
  preferences.end();
}

void loadCalibration() {
  preferences.begin("calibration", true);
  currentCO2Factor = preferences.getFloat("co2", 1.0f);
  currentSO2Factor = preferences.getFloat("so2", 1.0f);
  currentNO2Factor = preferences.getFloat("no2", 1.0f);
  preferences.end();
}

void enterSafeMode(const char* reason) {
  stopCooling();
  digitalWrite(heaterPin, LOW);
  Serial.print("CRITICAL: "); Serial.println(reason);
  char json[128]; snprintf(json, sizeof(json), "{\"error\":\"%s\"}", reason);
  ws.textAll(json);
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      char *msg = (char*)malloc(len + 1);
      if (!msg) return;
      memcpy(msg, data, len); msg[len] = '\0';
      auto parseField = [](const char* p, const char* f) -> float {
        const char* s = strstr(p, f); if(!s) return -1.0f;
        s += strlen(f); while(*s && (*s=='\"'||*s==' '||*s==':')) s++;
        return (*s=='\0'||*s==','||*s=='}') ? -1.0f : (float)atof(s);
      };
      float f1 = parseField(msg, "\"co2\""), f2 = parseField(msg, "\"so2\""), f3 = parseField(msg, "\"no2\"");
      xSemaphoreTake(factorMutex, portMAX_DELAY);
      if (f1 >= 0) currentCO2Factor = f1; if (f2 >= 0) currentSO2Factor = f2; if (f3 >= 0) currentNO2Factor = f3;
      xSemaphoreGive(factorMutex);
      if (f1 >= 0 || f2 >= 0 || f3 >= 0) saveCalibration(currentCO2Factor, currentSO2Factor, currentNO2Factor);
      free(msg);
    }
  }
}

void monteCarloTask(void *parameter) {
  while (true) {
    vTaskDelay(60000 / portTICK_PERIOD_MS);
    if (bufferFull) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(copyTemps, empiricalTemperatures, totalDataPoints * sizeof(float));
      memcpy(copyHums, empiricalHumidities, totalDataPoints * sizeof(float));
      memcpy(copyPress, empiricalPressures, totalDataPoints * sizeof(float));
      xSemaphoreGive(dataMutex);
      for (int i = 0; i < totalDataPoints; i++) {
        float dp = calculateDewPoint(copyTemps[i], copyHums[i]);
        rawDewPoints[i] = adjustDewPointForPressure(dp, copyPress[i]);
      }
      int lastPWM = 0;
      xSemaphoreTake(factorMutex, portMAX_DELAY);
      lastPWM = globalCurrentPWM;
      xSemaphoreGive(factorMutex);
      monteCarloSimulation(copyTemps, copyHums, copyPress, totalDataPoints, dataPointIndex, ambientRefTemp, lastPWM);
      lastCalTime = millis() / 1000;
    }
  }
}

void setup() {
  Serial.begin(115200);
  initializeSensors();
  performSelfTest();
  loadCalibration();
  initializeDashboard();

  dataMutex = xSemaphoreCreateMutex();
  factorMutex = xSemaphoreCreateMutex();

  ambientRefTemp = bme.readTemperature();
  float dp_est = calculateDewPoint(ambientRefTemp, bme.readHumidity());
  createCoolingProfile(dp_est, ambientRefTemp);
  verifyDewPointWithHeatingProfile();

  if (dataPointIndex > 1) {
    for (int i = 0; i < dataPointIndex; i++) {
        float dp = calculateDewPoint(empiricalTemperatures[i], empiricalHumidities[i]);
        rawDewPoints[i] = adjustDewPointForPressure(dp, empiricalPressures[i]);
    }
    monteCarloSimulation(empiricalTemperatures, empiricalHumidities, empiricalPressures, dataPointIndex, dataPointIndex, ambientRefTemp, globalCurrentPWM);
    lastCalTime = millis() / 1000;
    if (dataPointIndex >= totalDataPoints) bufferFull = true;
  }

  xTaskCreatePinnedToCore(monteCarloTask, "MCTask", 10000, NULL, 1, NULL, 1);
}

void loop() {
  if (!checkSensorHealth()) { enterSafeMode("Sensor Failure"); delay(5000); return; }
  float rt = bme.readTemperature(), rh = bme.readHumidity(), rp = bme.readPressure()/100.0f;
  filteredT = (emaAlphaT*rt)+(1-emaAlphaT)*filteredT;
  filteredH = (emaAlphaH*rh)+(1-emaAlphaH)*filteredH;
  filteredP = (emaAlphaP*rp)+(1-emaAlphaP)*filteredP;
  float dp = adjustDewPointForPressure(calculateDewPoint(filteredT, filteredH), filteredP);
  addRealTimeDataPoint(filteredT, filteredH, filteredP, dp);

  float cCO2, cSO2, cNO2;
  xSemaphoreTake(factorMutex, portMAX_DELAY);
  cCO2 = currentCO2Factor; cSO2 = currentSO2Factor; cNO2 = currentNO2Factor;
  xSemaphoreGive(factorMutex);

  float cdp = removeContaminantEffect(dp, cCO2, cSO2, cNO2, filteredT, ambientRefTemp, globalCurrentPWM);
  uint32_t fh = ESP.getFreeHeap();
  if (fh < minHeapSeen) minHeapSeen = fh;
  broadcastTelemetry(dp, cdp, cCO2, cSO2, cNO2, filteredT, filteredH, currentConfidence, coolingHealth, readBatteryVoltage(), readSupplyVoltage(), minHeapSeen, lastCalTime);
  delay(2000);
}

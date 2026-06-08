#include "config.h"
#include "Physics.h"
#include "Sensors.h"
#include "ThermalControl.h"
#include "Optimization.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include "three_js_bundle.h"

// ── Globals ──────────────────────────────────────────────────────────────────
Preferences preferences;
float filteredT = 25.0f, filteredH = 50.0f, filteredP = 1013.25f;
float currentCO2Factor = 1.0f, currentSO2Factor = 1.0f, currentNO2Factor = 1.0f;
float currentConfidence = 0.0f, coolingHealth = 1.0f, ambientRefTemp = 25.0f;
SemaphoreHandle_t dataMutex, factorMutex;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// HTML content (shortened for clarity in modular structure, using advanced version logic)
extern const char* htmlContent; // To be defined or kept here

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
      char *msg = (char*)malloc(len + 1); memcpy(msg, data, len); msg[len] = '\0';
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
      monteCarloSimulation(copyTemps, copyHums, copyPress, totalDataPoints, dataPointIndex, ambientRefTemp);
    }
  }
}

#include "three_js_bundle.h"
const char* htmlContent = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Advanced Dew Point Monitor</title>
    <script src="/three.min.js"></script>
    <style>
        body { margin: 0; overflow: hidden; background: #050505; color: #fff; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; }
        #overlay { position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none; }
        .panel { position: absolute; background: rgba(20, 20, 20, 0.85); backdrop-filter: blur(10px); padding: 20px; border: 1px solid rgba(255,255,255,0.1); pointer-events: auto; }
        #stats-panel { top: 20px; left: 20px; width: 300px; border-radius: 8px; border-left: 4px solid #00aaff; }
        #controls-panel { top: 20px; right: 20px; width: 220px; border-radius: 8px; }
        h1 { font-size: 1.2em; margin: 0 0 15px 0; color: #00aaff; text-transform: uppercase; letter-spacing: 2px; }
        .data-row { display: flex; justify-content: space-between; margin-bottom: 8px; font-family: monospace; font-size: 1.1em; }
        .label { color: #888; }
        .value { color: #00ffcc; }
        .unit { color: #555; font-size: 0.8em; margin-left: 4px; }
        hr { border: 0; border-top: 1px solid rgba(255,255,255,0.1); margin: 15px 0; }
        input { width: 100%; background: #111; color: #fff; border: 1px solid #333; padding: 5px; margin: 5px 0; border-radius: 4px; }
        button { width: 100%; cursor: pointer; background: #00aaff; color: #fff; border: none; padding: 10px; margin-top: 10px; border-radius: 4px; transition: 0.3s; font-weight: bold; }
        button:hover { background: #0088cc; box-shadow: 0 0 15px rgba(0,170,255,0.4); }
        #status-bar { position: absolute; bottom: 20px; left: 20px; color: #666; font-size: 0.8em; }
        .error { color: #ff3333 !important; }
    </style>
</head>
<body>
    <div id="overlay">
        <div id="stats-panel" class="panel">
            <h1>Environment</h1>
            <div class="data-row"><span class="label">Temperature</span><span class="value" id="val-t">--</span><span class="unit">°C</span></div>
            <div class="data-row"><span class="label">Humidity</span><span class="value" id="val-h">--</span><span class="unit">%</span></div>
            <hr>
            <h1>Dew Point</h1>
            <div class="data-row"><span class="label">Raw DP</span><span class="value" id="val-dp">--</span><span class="unit">°C</span></div>
            <div class="data-row"><span class="label">Corrected</span><span class="value" id="val-cdp" style="color:#00ff00">--</span><span class="unit">°C</span></div>
            <hr>
            <h1>System</h1>
            <div class="data-row"><span class="label">Fit Conf.</span><span class="value" id="val-conf">--</span><span class="unit">%</span></div>
            <div class="data-row"><span class="label">Cooling Health</span><span class="value" id="val-health">--</span><span class="unit">%</span></div>
            <hr>
            <h1>Diagnostics</h1>
            <div class="data-row"><span class="label">Free Heap</span><span class="value" id="val-heap">--</span><span class="unit">bytes</span></div>
            <div class="data-row"><span class="label">Uptime</span><span class="value" id="val-uptime">--</span><span class="unit">sec</span></div>
        </div>

        <div id="controls-panel" class="panel">
            <h1>Calibration</h1>
            CO2 Factor
            <input type="number" id="ico2" step="0.01" placeholder="CO2">
            SO2 Factor
            <input type="number" id="iso2" step="0.01" placeholder="SO2">
            NO2 Factor
            <input type="number" id="ino2" step="0.01" placeholder="NO2">
            <button onclick="updateCal()">Commit Calibration</button>
        </div>
        <div id="status-bar">WebSocket: <span id="ws-status">Disconnected</span> | Standalone Mode v3.1</div>
    </div>

    <script>
        let scene = new THREE.Scene();
        let camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        let renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setClearColor(0x050505, 1);
        renderer.setSize(window.innerWidth, window.innerHeight);
        document.body.appendChild(renderer.domElement);

        let grid = new THREE.GridHelper(100, 20, 0x00aaff, 0x222222);
        grid.rotation.x = Math.PI / 2;
        grid.position.z = -10;
        scene.add(grid);

        const pointsCount = 500;
        let rawGeo = new THREE.BufferGeometry();
        let corrGeo = new THREE.BufferGeometry();
        let rawPos = new Float32Array(pointsCount * 3);
        let corrPos = new Float32Array(pointsCount * 3);
        rawGeo.setAttribute('position', new THREE.BufferAttribute(rawPos, 3));
        corrGeo.setAttribute('position', new THREE.BufferAttribute(corrPos, 3));
        let rawPoints = new THREE.Points(rawGeo, new THREE.PointsMaterial({ color: 0xff3333, size: 0.4, transparent: true, opacity: 0.6 }));
        let corrPoints = new THREE.Points(corrGeo, new THREE.PointsMaterial({ color: 0x00ffcc, size: 0.6, transparent: true, opacity: 0.9 }));
        scene.add(rawPoints); scene.add(corrPoints);

        camera.position.set(0, 0, 40); camera.lookAt(0, 0, 0);

        let websocket;
        function initWebSocket() {
            websocket = new WebSocket(`ws://${window.location.hostname}/ws`);
            websocket.onopen = () => document.getElementById('ws-status').innerText = 'Connected';
            websocket.onclose = () => { document.getElementById('ws-status').innerText = 'Disconnected'; setTimeout(initWebSocket, 2000); };
            websocket.onmessage = onMessage;
        }

        function onMessage(event) {
            let data = JSON.parse(event.data);
            if (data.error) { document.getElementById('val-t').innerHTML = "ERROR"; document.getElementById('val-t').className = "value error"; return; }
            document.getElementById('val-t').innerText = data.t.toFixed(2);
            document.getElementById('val-h').innerText = data.h.toFixed(1);
            document.getElementById('val-dp').innerText = data.dp.toFixed(2);
            document.getElementById('val-cdp').innerText = data.cdp.toFixed(2);
            document.getElementById('val-conf').innerText = (data.conf * 100).toFixed(1);
            document.getElementById('val-health').innerText = (data.health * 100).toFixed(1);
            document.getElementById('val-heap').innerText = data.heap;
            document.getElementById('val-uptime').innerText = data.uptime;

            for (let i = 0; i < pointsCount - 1; i++) {
                rawPos[i * 3 + 1] = rawPos[(i + 1) * 3 + 1];
                corrPos[i * 3 + 1] = corrPos[(i + 1) * 3 + 1];
            }
            let x = ((pointsCount - 1) / 10.0) - 25.0;
            rawPos[(pointsCount - 1) * 3] = x; rawPos[(pointsCount - 1) * 3 + 1] = data.dp - 15;
            corrPos[(pointsCount - 1) * 3] = x; corrPos[(pointsCount - 1) * 3 + 1] = data.cdp - 15;
            rawGeo.attributes.position.needsUpdate = true; corrGeo.attributes.position.needsUpdate = true;
        }

        function updateCal() {
            websocket.send(JSON.stringify({
                co2: parseFloat(document.getElementById('ico2').value),
                so2: parseFloat(document.getElementById('iso2').value),
                no2: parseFloat(document.getElementById('ino2').value)
            }));
        }

        function animate() {
            requestAnimationFrame(animate);
            rawPoints.rotation.y = Math.sin(Date.now() * 0.0005) * 0.1;
            corrPoints.rotation.y = Math.sin(Date.now() * 0.0005) * 0.1;
            renderer.render(scene, camera);
        }
        window.onload = initWebSocket; animate();
    </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  initializeSensors();
  performSelfTest();
  loadCalibration();

  // Initialize in Access Point (AP) mode for standalone operation
  WiFi.softAP("DewPointMonitor", "12345678");
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", htmlContent); });
  server.on("/three.min.js", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncWebServerResponse *resp = r->beginResponse_P(200, "application/javascript", three_js_gz, three_js_gz_len);
    resp->addHeader("Content-Encoding", "gzip"); r->send(resp);
  });
  server.begin();
  dataMutex = xSemaphoreCreateMutex();
  factorMutex = xSemaphoreCreateMutex();
  ambientRefTemp = bme.readTemperature();
  float dp = calculateDewPoint(ambientRefTemp, bme.readHumidity());
  createCoolingProfile(dp, ambientRefTemp);
  verifyDewPointWithHeatingProfile();
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
  float cdp = removeContaminantEffect(dp, cCO2, cSO2, cNO2, filteredT, ambientRefTemp);

  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t uptime = millis() / 1000;

  char json[384];
  snprintf(json, sizeof(json), "{\"dp\":%.2f,\"cdp\":%.2f,\"co2\":%.3f,\"so2\":%.3f,\"no2\":%.3f,\"t\":%.2f,\"h\":%.1f,\"conf\":%.3f,\"health\":%.3f,\"heap\":%u,\"uptime\":%u}",
           dp, cdp, cCO2, cSO2, cNO2, filteredT, filteredH, currentConfidence, coolingHealth, freeHeap, uptime);
  ws.textAll(json);
  delay(2000);
}

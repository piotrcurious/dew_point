#include "Dashboard.h"
#include "three_js_bundle.h"
#include "Sensors.h"
#include <WiFi.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

extern void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

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
            <h1>Hardware</h1>
            <div class="data-row"><span class="label">Battery</span><span class="value" id="val-bvolt">--</span><span class="unit">V</span></div>
            <div class="data-row"><span class="label">Supply</span><span class="value" id="val-svolt">--</span><span class="unit">V</span></div>
            <hr>
            <h1>Diagnostics</h1>
            <div class="data-row"><span class="label">Free Heap</span><span class="value" id="val-heap">--</span><span class="unit">bytes</span></div>
            <div class="data-row"><span class="label">Min Heap</span><span class="value" id="val-minheap">--</span><span class="unit">bytes</span></div>
            <div class="data-row"><span class="label">Last Cal</span><span class="value" id="val-lastcal">--</span><span class="unit">sec ago</span></div>
            <div class="data-row"><span class="label">Uptime</span><span class="value" id="val-uptime">--</span><span class="unit">sec</span></div>
            <div class="data-row"><span class="label">Status</span><span class="value" id="val-status">IDLE</span></div>
        </div>

        <div id="controls-panel" class="panel">
            <h1>Calibration</h1>
            <div class="data-row"><span class="label">CO2:</span><span class="value" id="cur-co2">--</span></div>
            <div class="data-row"><span class="label">SO2:</span><span class="value" id="cur-so2">--</span></div>
            <div class="data-row"><span class="label">NO2:</span><span class="value" id="cur-no2">--</span></div>
            <hr>
            Manual Override:
            <input type="number" id="ico2" step="0.01" placeholder="CO2">
            <input type="number" id="iso2" step="0.01" placeholder="SO2">
            <input type="number" id="ino2" step="0.01" placeholder="NO2">
            <button onclick="updateCal()">Commit Factors</button>
            <button onclick="requestCal()" style="background:#555; margin-top:5px;">Force Optimize</button>
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
            document.getElementById('val-bvolt').innerText = data.bVolt.toFixed(2);
            document.getElementById('val-svolt').innerText = data.sVolt.toFixed(2);
            document.getElementById('val-heap').innerText = data.heap;
            document.getElementById('val-minheap').innerText = data.minHeap;
            document.getElementById('val-lastcal').innerText = data.lastCal;
            document.getElementById('val-uptime').innerText = data.uptime;
            document.getElementById('val-status').innerText = data.optimizing ? "OPTIMIZING" : "IDLE";
            document.getElementById('val-status').style.color = data.optimizing ? "#ffaa00" : "#00ffcc";

            document.getElementById('cur-co2').innerText = data.co2.toFixed(3);
            document.getElementById('cur-so2').innerText = data.so2.toFixed(3);
            document.getElementById('cur-no2').innerText = data.no2.toFixed(3);

            for (let i = 0; i < pointsCount - 1; i++) {
                rawPos[i * 3 + 1] = rawPos[(i + 1) * 3 + 1];
                corrPos[i * 3 + 1] = corrPos[(i + 1) * 3 + 1];
                // Ensure X stays fixed and correct for historical points
                let x = (i / 10.0) - 25.0;
                rawPos[i * 3] = x; corrPos[i * 3] = x;
            }
            let xFinal = ((pointsCount - 1) / 10.0) - 25.0;
            rawPos[(pointsCount - 1) * 3] = xFinal; rawPos[(pointsCount - 1) * 3 + 1] = data.dp - 15;
            corrPos[(pointsCount - 1) * 3] = xFinal; corrPos[(pointsCount - 1) * 3 + 1] = data.cdp - 15;
            rawGeo.attributes.position.needsUpdate = true; corrGeo.attributes.position.needsUpdate = true;
        }

        function updateCal() {
            websocket.send(JSON.stringify({
                co2: parseFloat(document.getElementById('ico2').value),
                so2: parseFloat(document.getElementById('iso2').value),
                no2: parseFloat(document.getElementById('ino2').value)
            }));
        }

        function requestCal() {
            websocket.send(JSON.stringify({ cmd: "calibrate" }));
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

void initializeDashboard() {
  WiFi.softAP("DewPointMonitor", "12345678");
  Serial.print("AP IP Address: "); Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", htmlContent); });
  server.on("/three.min.js", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncWebServerResponse *resp = r->beginResponse_P(200, "application/javascript", three_js_gz, three_js_gz_len);
    resp->addHeader("Content-Encoding", "gzip"); r->send(resp);
  });
  server.begin();
}

void broadcastTelemetry(float dp, float cdp, float cCO2, float cSO2, float cNO2, float t, float h, float conf, float health, float bVolt, float sVolt, uint32_t minFreeHeap, uint32_t lastCal, bool isOptimizing) {
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t uptime = millis() / 1000;
  uint32_t ago = (lastCal == 0) ? 0 : uptime - lastCal;
  char json[640];
  snprintf(json, sizeof(json), "{\"dp\":%.2f,\"cdp\":%.2f,\"co2\":%.3f,\"so2\":%.3f,\"no2\":%.3f,\"t\":%.2f,\"h\":%.1f,\"conf\":%.3f,\"health\":%.3f,\"heap\":%u,\"minHeap\":%u,\"uptime\":%u,\"bVolt\":%.2f,\"sVolt\":%.2f,\"lastCal\":%u,\"optimizing\":%d}",
           dp, cdp, cCO2, cSO2, cNO2, t, h, conf, health, freeHeap, minFreeHeap, uptime, bVolt, sVolt, ago, isOptimizing ? 1 : 0);
  ws.textAll(json);
}

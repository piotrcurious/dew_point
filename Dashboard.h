#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "config.h"
#include <ESPAsyncWebServer.h>

extern AsyncWebServer server;
extern AsyncWebSocket ws;

void initializeDashboard();
void broadcastTelemetry(float dp, float cdp, float cCO2, float cSO2, float cNO2, float t, float h, float conf, float health, float bVolt, float sVolt, uint32_t minFreeHeap);

#endif

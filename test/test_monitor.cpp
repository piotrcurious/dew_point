#include "arduino_mock.h"

SerialMock Serial;
TwoWire Wire;
MockState g_mock;
WiFiStub WiFi;

#define private public
#define protected public

#include "../dew_point_monitor.ino"

#undef private
#undef protected

int main() {
    std::cout << "Starting Arduino Mock Test (ESP32 Targeting)..." << std::endl;
    setup();
    std::cout << "Setup complete. Running loop for a few iterations..." << std::endl;
    for(int i=0; i<100; ++i) {
        loop();
        // Manually trigger the background task logic if needed,
        // or just let the task stub represent its creation.
        if (i == 50) {
            std::cout << "[Mock] Simulating background MC refinement..." << std::endl;
            // In a real multi-threaded mock we'd have a thread running monteCarloTask
            // For now, let's just call it once.
            // monteCarloSimulation(empiricalTemperatures, empiricalHumidities, empiricalPressures, totalDataPoints);
        }
    }
    std::cout << "Test finished." << std::endl;
    return 0;
}

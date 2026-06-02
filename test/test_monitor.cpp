#include "arduino_mock.h"

SerialMock Serial;
TwoWire Wire;
MockState g_mock;

#define private public
#define protected public

#include "../dew_point_monitor.ino"

#undef private
#undef protected

int main() {
    std::cout << "Starting Arduino Mock Test..." << std::endl;
    setup();
    std::cout << "Setup complete. Running loop..." << std::endl;
    loop();
    std::cout << "Test finished." << std::endl;
    return 0;
}

#include "Physics.cpp"
#include <iostream>

int main() {
    float T = 10.0;
    float ambientT = 25.0;
    int PWM = 0;

    std::cout << "CO2 Factor: " << getTempAdsorptionFactor(T, ambientT, PWM, co2TempCoeff) << std::endl;
    std::cout << "SO2 Factor: " << getTempAdsorptionFactor(T, ambientT, PWM, so2TempCoeff) << std::endl;
    std::cout << "NO2 Factor: " << getTempAdsorptionFactor(T, ambientT, PWM, no2TempCoeff) << std::endl;

    return 0;
}

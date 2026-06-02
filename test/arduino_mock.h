#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdint>
#include <random>

// --- Arduino Constants ---
#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define SHT4X_HIGH_PRECISION 0
#define SHT4X_NO_HEATER 0
#define SHT4X_HEATER_MED_100MS 1

// --- Arduino Types ---
typedef uint8_t byte;
typedef bool boolean;

// Magnus-Tetens helper for mock
inline float mock_calculateDewPoint(float temperature, float humidity) {
  if (humidity <= 0.0f) humidity = 0.01f;
  float alpha = ((17.27f * temperature) / (237.7f + temperature)) + log(humidity / 100.0f);
  return (237.7f * alpha) / (17.27f - alpha);
}

// --- Mock Global State ---
struct MockState {
    float bme_temp = 25.0f;
    float bme_hum = 50.0f;
    float bme_press = 101325.0f;
    float sht_temp = 25.0f;
    float sht_hum = 50.0f;
    unsigned long mock_millis = 0;
    int cooler_pwm = 0;

    // Hidden "real" contamination for testing Monte Carlo
    float true_co2_factor = 1.05f;
    float true_so2_factor = 1.02f;
    float true_no2_factor = 1.03f;

    std::mt19937 gen{42};
    std::normal_distribution<float> noise{0.0f, 0.02f};
};
extern MockState g_mock;

// --- Arduino Functions ---
inline void delay(unsigned long ms) {
    g_mock.mock_millis += ms;

    float ambient = 25.0f;
    int steps = ms / 10; // Finer simulation steps
    if (steps == 0) steps = 1;
    float dt = (float)ms / steps / 1000.0f; // in seconds

    for(int i=0; i<steps; ++i) {
        // More realistic cooling: Peltier efficiency drops as deltaT increases
        float deltaT = g_mock.bme_temp - ambient;
        float cooling_power = (g_mock.cooler_pwm / 255.0f) * 2.0f;
        float heat_leak = 0.1f * (ambient - g_mock.bme_temp);

        // Simplified thermal mass: dT = (Power / Mass) * dt
        g_mock.bme_temp += (heat_leak - cooling_power) * dt;
    }
}

inline unsigned long millis() {
    return g_mock.mock_millis;
}

inline void pinMode(int pin, int mode) {}
inline void digitalWrite(int pin, int val) {}
inline int digitalRead(int pin) { return LOW; }
inline void analogWrite(int pin, int val) {
    if (pin == 9) g_mock.cooler_pwm = val;
}

template<typename T>
inline T constrain(T x, T a, T b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

// --- Serial Mock ---
class SerialMock {
public:
    void begin(unsigned long baud) {}
    void print(const char* s) { std::cout << s; }
    void print(float f, int p = 2) { std::cout << f; }
    void print(int i) { std::cout << i; }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(float f, int p = 2) { std::cout << f << std::endl; }
    void println(int i) { std::cout << i << std::endl; }
    void println() { std::cout << std::endl; }
};
extern SerialMock Serial;

// --- Adafruit_Sensor Mock ---
struct sensors_event_t {
    float temperature;
    float relative_humidity;
    float pressure;
};

// --- BME280 Mock ---
class Adafruit_BME280 {
public:
    bool begin(uint8_t addr) { return true; }
    float readTemperature() {
        return g_mock.bme_temp + g_mock.noise(g_mock.gen);
    }
    float readHumidity() {
        float base_dp = mock_calculateDewPoint(g_mock.bme_temp, g_mock.bme_hum);

        // Apply "true" contaminant effects to the dew point
        float adj_co2 = 0.0025f * log(1.0f + 0.4f * g_mock.true_co2_factor);
        float adj_so2 = 0.008f * powf(g_mock.true_so2_factor, 2.0f) * 0.9f;
        float adj_no2 = 0.005f * expf(1.2f * g_mock.true_no2_factor);

        float affected_dp = base_dp + base_dp * (adj_co2 + adj_so2 + adj_no2);

        float A = 17.27f;
        float B = 237.7f;
        float alpha_dp = (A * affected_dp) / (B + affected_dp);
        float alpha_t = (A * g_mock.bme_temp) / (B + g_mock.bme_temp);

        float rh = 100.0f * expf(alpha_dp - alpha_t);
        if (rh > 100.0f) rh = 100.0f;

        return rh + g_mock.noise(g_mock.gen);
    }
    float readPressure() { return g_mock.bme_press; }
};

// --- SHT4x Mock ---
class Adafruit_SHT4x {
public:
    bool begin() { return true; }
    void setPrecision(int p) {}
    void setHeater(int h) {
        if (h == SHT4X_HEATER_MED_100MS) {
            g_mock.sht_temp += 0.5f;
        }
    }
    void getEvent(sensors_event_t* humidity, sensors_event_t* temp) {
        temp->temperature = g_mock.sht_temp + g_mock.noise(g_mock.gen);
        humidity->relative_humidity = g_mock.sht_hum + g_mock.noise(g_mock.gen);
        g_mock.sht_temp -= 0.01f;
        if (g_mock.sht_temp < 25.0f) g_mock.sht_temp = 25.0f;
    }
};

// --- Wire Mock ---
class TwoWire {
public:
    void begin() {}
};
extern TwoWire Wire;

#endif

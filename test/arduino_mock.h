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
#include <mutex>

// --- Arduino/ESP32 Constants ---
#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define SHT4X_HIGH_PRECISION 0
#define SHT4X_NO_HEATER 0
#define SHT4X_HEATER_MED_100MS 1

// --- Types ---
typedef uint8_t byte;
typedef bool boolean;

// --- FreeRTOS Stubs ---
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
#define portTICK_PERIOD_MS 1
#define pdTRUE true
#define pdFALSE false
#define portMAX_DELAY 0xFFFFFFFF

inline void vTaskDelay(uint32_t ticks) {
    // In mock, we can just increment mock millis
}

typedef void (*TaskFunction_t)(void*);

inline void xTaskCreatePinnedToCore(TaskFunction_t func, const char* name, uint32_t stack, void* param, int priority, TaskHandle_t* handle, int core) {
    // In a real mock we might start a thread, but for simplicity we'll just track it
    // or run it occasionally.
    std::cout << "[Mock] Task Created: " << name << " on Core " << core << std::endl;
}

inline SemaphoreHandle_t xSemaphoreCreateMutex() {
    return (SemaphoreHandle_t)new std::mutex();
}

inline bool xSemaphoreTake(SemaphoreHandle_t xSemaphore, uint32_t xBlockTime) {
    ((std::mutex*)xSemaphore)->lock();
    return true;
}

inline void xSemaphoreGive(SemaphoreHandle_t xSemaphore) {
    ((std::mutex*)xSemaphore)->unlock();
}

// --- WebSocket/Web Stubs ---
enum AwsEventType { WS_EVT_CONNECT, WS_EVT_DISCONNECT, WS_EVT_PONG, WS_EVT_ERROR, WS_EVT_DATA };
class AsyncWebSocketClient {};
class AsyncWebSocket {
public:
    AsyncWebSocket(const char* url) {}
    void textAll(const char* msg) {}
    void cleanupClients() {}
    void onEvent(void (*f)(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType, void*, uint8_t*, size_t)) {}
};

class AsyncWebServerRequest {
public:
    void send(int code, const char* type, const char* content) {}
};

class AsyncWebServer {
public:
    AsyncWebServer(int port) {}
    void begin() {}
    void addHandler(AsyncWebSocket* ws) {}
    void on(const char* url, int method, void (*f)(AsyncWebServerRequest*)) {}
};

#define HTTP_GET 0

class WiFiStub {
public:
    void begin(const char* ssid, const char* pass) {}
};
extern WiFiStub WiFi;

// --- Arduino Functions ---
template<typename T>
inline T constrain(T x, T a, T b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

// Magnus-Tetens helper for mock
inline float mock_calculateDewPoint(float temperature, float humidity) {
  if (humidity <= 0.0f) humidity = 0.01f;
  float alpha = ((17.27f * temperature) / (237.7f + temperature)) + log(humidity / 100.0f);
  return (237.7f * alpha) / (17.27f - alpha);
}

// --- Mock Global State ---
struct MockState {
    float ambient_temp = 25.0f;
    float ambient_hum = 50.0f;
    float bme_press = 1013.25f;

    // Physical state
    float peltier_cold_temp = 25.0f;
    float peltier_hot_temp = 25.0f;
    float sensor_temp = 25.0f;

    // Electrical state
    float v_supply = 12.0f;
    float mosfet_gate_v = 0.0f;
    float actual_current = 0.0f;
    float mosfet_temp = 25.0f;

    // Thermal properties
    float thermal_mass_cold = 10.0f;
    float thermal_mass_hot = 50.0f;
    float thermal_mass_sensor = 0.5f;
    float thermal_coupling_sensor = 0.8f;
    float thermal_resistance_hot_ambient = 0.1f;
    float thermal_resistance_cold_ambient = 200.0f;

    float bme_hum = 50.0f;
    float bme_press_val = 1013.25f;
    float bme_temp_val = 25.0f;
    float bme_hum_val = 50.0f;

    float sht_temp = 25.0f;
    float sht_hum = 50.0f;

    unsigned long mock_millis = 0;
    int cooler_pwm = 0;

    // Contamination concentrations
    float true_co2 = 0.8f;
    float true_so2 = 0.4f;
    float true_no2 = 0.2f;

    std::mt19937 gen{42};
    std::normal_distribution<float> noise{0.0f, 0.005f};
};
extern MockState g_mock;

inline void delay(unsigned long ms) {
    g_mock.mock_millis += ms;

    const float dt = 0.001f; // 1ms
    for(unsigned long i=0; i<ms; ++i) {
        // --- Electrical Model (MOSFET + Peltier) ---
        // MOSFET Gate Drive Simulation
        float target_gate_v = (g_mock.cooler_pwm > 0) ? 5.0f : 0.0f;
        float gate_tc = 0.0001f; // 0.1ms gate charge time
        g_mock.mosfet_gate_v += (target_gate_v - g_mock.mosfet_gate_v) * (dt / gate_tc);

        // Drain Current (simplified)
        float v_threshold = 2.0f;
        float k_mosfet = 2.0f; // A/V^2
        float i_drain = 0.0f;
        if (g_mock.mosfet_gate_v > v_threshold) {
            i_drain = k_mosfet * powf(g_mock.mosfet_gate_v - v_threshold, 2.0f);
        }

        // Peltier Resistance Limit
        float p_resistance = 2.0f;
        float i_max = g_mock.v_supply / p_resistance;
        if (i_drain > i_max) i_drain = i_max;

        // Supply Droop
        float r_supply = 0.1f;
        float current_v_supply = 12.0f - i_drain * r_supply;
        i_drain = current_v_supply / (p_resistance + 0.1f); // Adjust current with new supply V and RDSon

        g_mock.actual_current = i_drain;

        // MOSFET Losses (Switching + Conduction)
        float v_ds = current_v_supply - i_drain * p_resistance;
        float p_mosfet = i_drain * v_ds;
        g_mock.mosfet_temp += (p_mosfet - (g_mock.mosfet_temp - g_mock.ambient_temp)/2.0f) * dt / 1.0f;

        // --- Peltier Thermal Model ---
        float alpha_seebeck = 0.05f;
        float K_peltier = 0.1f;
        float Tc_k = g_mock.peltier_cold_temp + 273.15f;
        float Th_k = g_mock.peltier_hot_temp + 273.15f;

        float Qp = alpha_seebeck * i_drain * Tc_k - 0.5f * i_drain * i_drain * p_resistance - K_peltier * (g_mock.peltier_hot_temp - g_mock.peltier_cold_temp);
        float Qh = alpha_seebeck * i_drain * Th_k + 0.5f * i_drain * i_drain * p_resistance - K_peltier * (g_mock.peltier_hot_temp - g_mock.peltier_cold_temp);

        float airflow = 0.01f * fabsf(g_mock.sensor_temp - g_mock.ambient_temp) + (g_mock.cooler_pwm / 255.0f) * 0.2f;

        float Q_leak_cold = (g_mock.ambient_temp - g_mock.peltier_cold_temp) / g_mock.thermal_resistance_cold_ambient;
        float current_R_hot = 0.1f / (1.0f + airflow * 10.0f);
        float Q_dissipate_hot = (g_mock.peltier_hot_temp - g_mock.ambient_temp) / current_R_hot;
        float Q_sensor_coupling = g_mock.thermal_coupling_sensor * (g_mock.peltier_cold_temp - g_mock.sensor_temp);

        g_mock.peltier_cold_temp += (-Qp + Q_leak_cold - Q_sensor_coupling) / g_mock.thermal_mass_cold * dt;
        g_mock.peltier_hot_temp += (Qh - Q_dissipate_hot) / g_mock.thermal_mass_hot * dt;
        g_mock.sensor_temp += (Q_sensor_coupling) / g_mock.thermal_mass_sensor * dt;
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
        return g_mock.sensor_temp + g_mock.noise(g_mock.gen);
    }
    float readHumidity() {
        float base_dp = mock_calculateDewPoint(g_mock.ambient_temp, g_mock.ambient_hum);
        float temp_factor = 1.0f + 0.05f * (g_mock.ambient_temp - g_mock.sensor_temp);
        float adj_co2 = 0.0025f * logf(1.0f + 0.4f * g_mock.true_co2) * temp_factor;
        float adj_so2 = 0.008f * powf(g_mock.true_so2, 2.0f) * 0.9f * temp_factor;
        float adj_no2 = 0.005f * (expf(1.2f * g_mock.true_no2) - 1.0f) * temp_factor;
        float affected_dp = base_dp + base_dp * (adj_co2 + adj_so2 + adj_no2);
        float A = 17.27f;
        float B = 237.7f;
        float alpha_dp = (A * affected_dp) / (B + affected_dp);
        float alpha_t = (A * g_mock.sensor_temp) / (B + g_mock.sensor_temp);
        float rh = 100.0f * expf(alpha_dp - alpha_t);
        if (rh > 100.0f) rh = 100.0f;
        if (rh < 0.0f) rh = 0.01f;
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

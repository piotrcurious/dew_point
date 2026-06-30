#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <cstring>

// Mock Arduino Constants
#define OUTPUT 0x1
#define INPUT 0x0
#define HIGH 0x1
#define LOW 0x0

// Mock ESP32 Constants
#define portTICK_PERIOD_MS 1
#define portMAX_DELAY 0xFFFFFFFF

// Basic Arduino Functions
inline unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

inline void delay(unsigned long ms) {
    // std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline void pinMode(int pin, int mode) {}
inline void digitalWrite(int pin, int val) {}
inline int analogRead(int pin) { return 2048; }
inline void analogWrite(int pin, int val) {}

// Mock Serial
class MockSerial {
public:
    void begin(int baud) {}
    void print(const char* s) { std::cout << s; }
    void print(float f) { std::cout << f; }
    void print(int i) { std::cout << i; }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(float f) { std::cout << f << std::endl; }
    void println(int i) { std::cout << i << std::endl; }
    void println() { std::cout << std::endl; }
};
extern MockSerial Serial;

// Mock FreeRTOS
typedef std::recursive_mutex* SemaphoreHandle_t;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return new std::recursive_mutex(); }
inline void xSemaphoreTake(SemaphoreHandle_t s, unsigned long delay) { if(s) s->lock(); }
inline void xSemaphoreGive(SemaphoreHandle_t s) { if(s) s->unlock(); }

typedef void (*TaskFunction_t)(void*);
inline void xTaskCreatePinnedToCore(TaskFunction_t f, const char* name, int stack, void* param, int prio, void** handle, int core) {
}
inline void vTaskDelay(unsigned long ticks) {
}

// Mock Preferences
class Preferences {
public:
    bool begin(const char* name, bool readOnly) { return true; }
    void putFloat(const char* key, float val) { storage[key] = val; }
    float getFloat(const char* key, float def) {
        if (storage.find(key) == storage.end()) return def;
        return storage[key];
    }
    void end() {}
private:
    std::map<std::string, float> storage;
};

// Mock BME280
class Adafruit_BME280 {
public:
    bool begin(int addr) { return true; }
    float readTemperature() { return temp; }
    float readHumidity() { return hum; }
    float readPressure() { return press * 100.0f; }
    float temp = 25.0f;
    float hum = 50.0f;
    float press = 1013.25f;
};

// Mock SHT4x
#define SHT4X_HIGH_PRECISION 0
#define SHT4X_NO_HEATER 0
#define SHT4X_HEATER_MED_100MS 1

struct sensors_event_t {
    float temperature;
    float relative_humidity;
};

class Adafruit_SHT4x {
public:
    bool begin() { return true; }
    void setPrecision(int p) {}
    void setHeater(int h) {}
    void getEvent(sensors_event_t* h, sensors_event_t* t) {
        t->temperature = temp;
        h->relative_humidity = hum;
    }
    float temp = 25.0f;
    float hum = 50.0f;
};

// Mock WiFi/AsyncServer
class AsyncWebSocketClient {};
enum AwsEventType { WS_EVT_CONNECT, WS_EVT_DISCONNECT, WS_EVT_PONG, WS_EVT_ERROR, WS_EVT_DATA };
struct AwsFrameInfo { bool final; size_t index; size_t len; };
class AsyncWebSocket {
public:
    AsyncWebSocket(const char* url) {}
    void onEvent(void (*f)(void*, void*, int, void*, uint8_t*, size_t)) {}
    void textAll(const char* s) {}
};
class AsyncWebServerRequest {
public:
    void send(int code, const char* type, const char* content) {}
};
class AsyncWebServerResponse {
public:
    void addHeader(const char* k, const char* v) {}
};
class AsyncWebServer {
public:
    AsyncWebServer(int port) {}
    void addHandler(void* h) {}
    void on(const char* url, int method, void (*f)(AsyncWebServerRequest*)) {}
    void begin() {}
};
#define HTTP_GET 0

// Mock ESP
class MockESP {
public:
    uint32_t getFreeHeap() { return 100000; }
};
extern MockESP ESP;

#endif

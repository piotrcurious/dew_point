#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"
#include <Adafruit_BME280.h>
#include <Adafruit_SHT4x.h>

extern Adafruit_BME280 bme;
extern Adafruit_SHT4x sht4x;

void initializeSensors();
bool checkSensorHealth();
void performSelfTest();

#endif

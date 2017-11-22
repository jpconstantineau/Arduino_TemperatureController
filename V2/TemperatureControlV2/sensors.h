#ifndef SENSORS_H_
#define SENSORS_H_
#include "configuration.h" // 
#include <OneWire.h>           // Comms for DallasTemperature
#include <DallasTemperature.h> // DS18B20 Library

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
extern OneWire oneWire;
extern DallasTemperature sensors;
extern DeviceAddress ThermometerAddress[NUMBER_OF_SENSORS] ;
extern double  Temperature[NUMBER_OF_SENSORS];
// arrays to hold device addresses
extern int sensorcount ;
extern int inputselected ;
extern int Ambientselected;
extern int Elementselected;
extern DeviceAddress Thermometer0, Thermometer1, Thermometer2;//, Thermometer3;
extern double Temperature0,Temperature1,Temperature2;//,Temperature3;


void UpdateSensors();
void SetupSensors();
#endif

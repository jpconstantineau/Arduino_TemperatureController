#include "sensors.h"
#include "configuration.h"
#include "control.h"
#include "display.h"

OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int sensorcount ;
int inputselected ;
int Ambientselected;
int Elementselected;
DeviceAddress ThermometerAddress[NUMBER_OF_SENSORS] ;
double  Temperature[NUMBER_OF_SENSORS];

void SetupSensors()
{
  sensorcount =0;

Ambientselected = 1;
Elementselected = 0;
  // setting up Dallas Sensors
  sensors.begin();
  sensorcount =  sensors.getDeviceCount();
  sensors.setWaitForConversion(false);
 // String sensorVal = String(sensorcount);
//  sensorVal.toCharArray(sensorPrintout, 4);

//   TFTscreen.setCursor(0, 00); 
 //  TFTscreen.print("Dallas: ");
//   TFTscreen.println(sensorcount);
 
  if (sensorcount)
  {
    for(int DSid=0; DSid<sensorcount; DSid++)
    {
    if (sensors.getAddress(ThermometerAddress[DSid], DSid)) {sensors.setResolution(ThermometerAddress[DSid], TEMPERATURE_PRECISION); printAddress(ThermometerAddress[DSid]);TFTscreen.println(" "); }  
    }
    

   
  } else
  { 
    TFTscreen.println("DS18B20 not present");  
  }
}

void UpdateSensors()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
    {
      PIDData[PIDid]->Input = sensors.getTempC(PIDData[PIDid]->deviceAddress);
     }
     
   for(int DSid=0; DSid<sensorcount; DSid++)
    {
      Temperature[DSid] = sensors.getTempC(ThermometerAddress[DSid]);
    } 
     sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
 
 
}



#include "configuration.h"
#include "display.h"
#include "menu.h"
#include "sensors.h"
#include "control.h"
#include "io.h"
#include <Wire.h>
#include "RTClib.h"
#define _sclk 13
#define _miso 12
#define _mosi 11
#define _cs 14
#define _dc 15
#define _rst 13
long rotaryHalfSteps_last;
long lastrotaryInput;

int relay[4] = {19, 20,21,22};
RTC_DS1307 rtc;

DateTime now;

boolean inmenu;
//Adafruit_ILI9340 tft = Adafruit_ILI9340(cs, dc, rst);
// ------------------------------------------------------
void setup() { 
    Serial.begin(BAUDRATE);   // initialize the serial port
    SetupRotaryEncoder(); 

    rotaryHalfSteps_last = rotaryHalfSteps;
   


  rtc.begin();
   if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
    SetupDisplay();           // initialize the display
    SetupMenu();              
    SetupOutputPins();        // Setting up Pins
    SetupSensors();
    Serial.print("Setup PIDs");
    SetupPIDs();              //  Setting up PID
    lastrotaryInput = 0;
    delay(SETUPDELAY);
    inmenu = false;
    Serial.print("Setup Complete");
    TFTscreen.fillScreen(ILI9340_BLACK);
}

// ------------------------------------------------------
void loop() {
now = rtc.now();
   //RunMenu();
   DisplayMenu();
   UpdateSensors();
   UpdateTrends();
   UpdateDisplayLabels();
   UpdateDisplayDiv(); 
   UpdateDisplayValues();
   UpdateDisplayMode();
   RunPID();
   DriveOutput();
 
   if (rotaryHalfSteps_last < rotaryHalfSteps)
   {
    rotaryHalfSteps_last = rotaryHalfSteps;
    lastrotaryInput = millis();
    switch (mState)
     {
         case mSTART:
         ms.next(); inmenu = true;
         break;
        case mSP:    
         doublevalue = doublevalue + 0.1;
        break;
      case mOP:    
         doublevalue = doublevalue + 1; if (doublevalue >100) {doublevalue =100;}
        break;
      case mTUNE_P:
               doublevalue = doublevalue + 1;
        break;
      case mTUNE_I:        
               doublevalue = doublevalue + 0.1;
        break;
      case mTUNE_D:        
               doublevalue = doublevalue + 0.1;
        break;
        case   mSELECTSENSOR:        
            SelectedAddressid = SelectedAddressid+1; if (SelectedAddressid >= sensorcount) {SelectedAddressid = 0;};
        break;
     }    
   }
   if (rotaryHalfSteps_last > rotaryHalfSteps)
   {
    rotaryHalfSteps_last = rotaryHalfSteps;
    lastrotaryInput = millis();
    switch (mState)
     {
         case mSTART:
         ms.prev(); inmenu = true;
         break;
        case mSP:    
         doublevalue = doublevalue - 0.1;
        break;
      case mOP:    
         doublevalue = doublevalue - 1; if (doublevalue <0) {doublevalue =0;} ;
        break;
      case mTUNE_P:
               doublevalue = doublevalue - 1;
        break;
      case mTUNE_I:        
               doublevalue = doublevalue - 0.1;
        break;
      case mTUNE_D:        
               doublevalue = doublevalue - 0.1;
        break;
        case   mSELECTSENSOR:        
            SelectedAddressid = SelectedAddressid-1; if (SelectedAddressid <0) {SelectedAddressid = sensorcount-1;};
        break;
     } 

   }
   if( buttonPressedHeld(20))
   {
    lastrotaryInput = millis();
    switch (mState)
     {
         case mSTART:
            ms.select(); inmenu = true;
         break;
        case mSP:    
            mState = mSTART; PIDData[Menu_PIDid]->Setpoint = doublevalue; SavePIDStruct(Menu_PIDid, *PIDData[Menu_PIDid]);
        break;
      case mOP:    
            mState = mSTART; PIDData[Menu_PIDid]->Output = doublevalue * PIDData[Menu_PIDid]->WindowSize/100;SavePIDStruct(Menu_PIDid, *PIDData[Menu_PIDid]);
        break;
      case mTUNE_P:
            mState = mSTART; PIDData[Menu_PIDid]->Kp = doublevalue;SavePIDStruct(Menu_PIDid, *PIDData[Menu_PIDid]);
        break;
      case mTUNE_I:        
            mState = mSTART; PIDData[Menu_PIDid]->Ki = doublevalue;SavePIDStruct(Menu_PIDid, *PIDData[Menu_PIDid]);
        break;
      case mTUNE_D:        
            mState = mSTART; PIDData[Menu_PIDid]->Kd = doublevalue;SavePIDStruct(Menu_PIDid, *PIDData[Menu_PIDid]);
        break;
      case   mSELECTSENSOR:        
            mState = mSTART; 
            PIDData[Menu_PIDid]->deviceAddress[0] = ThermometerAddress[SelectedAddressid][0]; 
            PIDData[Menu_PIDid]->deviceAddress[1] = ThermometerAddress[SelectedAddressid][1]; 
            PIDData[Menu_PIDid]->deviceAddress[2] = ThermometerAddress[SelectedAddressid][2]; 
            PIDData[Menu_PIDid]->deviceAddress[3] = ThermometerAddress[SelectedAddressid][3]; 
            PIDData[Menu_PIDid]->deviceAddress[4] = ThermometerAddress[SelectedAddressid][4]; 
            PIDData[Menu_PIDid]->deviceAddress[5] = ThermometerAddress[SelectedAddressid][5]; 
            PIDData[Menu_PIDid]->deviceAddress[6] = ThermometerAddress[SelectedAddressid][6]; 
            PIDData[Menu_PIDid]->deviceAddress[7] = ThermometerAddress[SelectedAddressid][7]; 
            SavePIDStruct(Menu_PIDid, *PIDData[Menu_PIDid]);
        break;
     } 
    }
   if (inmenu){
   if ((millis() - lastrotaryInput) > MENUTIMEOUT)  // return to RUN after 3 seconds idle
      {
        inmenu = false;
        ms.reset();
        mState = mSTART;
      }
    }
} // LOOP ALL DONE -----------------------------------------



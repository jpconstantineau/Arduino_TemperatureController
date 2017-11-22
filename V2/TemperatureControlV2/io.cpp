#include "io.h"
#include "configuration.h"
#include <arduino.h>
#include "control.h"

#include "display.h"
#include "configuration.h"
#include <EEPROM.h>

unsigned long lastInput; // last button press

// 'threshold' is the De-bounce Adjustment factor for the Rotary Encoder.
// The threshold value I'm using limits it to 100 half pulses a second
volatile unsigned long threshold = 10000;


// 'rotaryHalfSteps' is the counter of half-steps. The actual
// number of steps will be equal to rotaryHalfSteps / 2
volatile long rotaryHalfSteps = 0;

volatile unsigned long int0time = 0;
volatile unsigned long int1time = 0;
volatile uint8_t int0signal = 0;
volatile uint8_t int1signal = 0;
volatile uint8_t int0history = 0;
volatile uint8_t int1history = 0;

   unsigned char buttonState;
    unsigned long buttonTimer;

void int0()
  {
  if ( micros() - int0time < threshold )
    return;
  int0history = int0signal;
  int0signal = bitRead(PIND,2);
  if ( int0history==int0signal )
    return;
  int0time = micros();
  if ( int0signal == int1signal )
    rotaryHalfSteps++;
  else
    rotaryHalfSteps--;
  }

void int1()
  {
  if ( micros() - int1time < threshold )
    return;
  int1history = int1signal;
  int1signal = bitRead(PIND,3);
  if ( int1history==int1signal )
    return;
  int1time = micros();
  }

void SetupRotaryEncoder()
{
  digitalWrite(ROTARYENC0_PIN, HIGH);
  digitalWrite(ROTARYENC1_PIN, HIGH);

  attachInterrupt(0, int0, CHANGE);
  attachInterrupt(1, int1, CHANGE);

  pinMode(ROTARYBUTTON_PIN, INPUT);
  digitalWrite(ROTARYBUTTON_PIN, HIGH);  
}







bool buttonPressedHeld(short delay_millis) {
  // Is the button closed (being pressed)?  
  if (!bitRead(PIND,4)) {
    // If this is the first time we've checked the button, set the state and start the timer
    if (buttonState != BUTTON_PRESSED) {
      buttonState = BUTTON_PRESSED;
      buttonTimer = millis();
    }
    else {
      // Otherwise, check the timer to see if it's expired
      if (millis() - buttonTimer > delay_millis) {
        // the wait timer has expired, reset the button state
        buttonState = BUTTON_RESET;
        // and return true indicating the button has been held down for long enough
        return true;
      }
    }
  }
  else {
    // button is open (not being pressed) was it already pressed?
    if (buttonState == BUTTON_PRESSED) {
      // it was pressed and released too quickly, reset it
      buttonState = BUTTON_RESET;
    }
  }

  return false;
}




// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output

     for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
    {
        if((now - windowStartTime[PIDid])>(PIDData[PIDid]->WindowSize))
        { //time to shift the Relay Window
           windowStartTime[PIDid] += PIDData[PIDid]->WindowSize;
        }
        if((onTime[PIDid] > 1000) && (onTime[PIDid] > (now - windowStartTime[PIDid])))
        {
           digitalWrite(relay[PIDid],HIGH);
           Output_Status[PIDid] = true;
        }
        else
        {
           digitalWrite(relay[PIDid],LOW);
           Output_Status[PIDid] = false;
        }
    }    
}





void  SetupOutputPins()  
{
  lastInput = 0; // last button press
  pinMode(pwm, OUTPUT);
  
  pinMode(10, OUTPUT);  
  analogWrite(pwm, 128);
  for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
  {
    pinMode(relay[PIDid], OUTPUT);
    digitalWrite(relay[PIDid], LOW);  // make sure it is off to start
  }
}








// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
/*void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
   
   if (trendInterval != EEPROM_readLong(TrendAddress))
   {
      EEPROM_writeLong(TrendAddress, trendInterval);
   }
   if (tickInterval != EEPROM.read(TickAddress))
   {
      EEPROM_writeLong(TickAddress, tickInterval);
   }

}*/

void LoadPIDStruct(int PIDid,PIDStruct* PIDData){
int address = CONFIG_START + PIDid *sizeof(default_PID_settings);
Serial.print("Loading from ");
Serial.print(address);
Serial.print(" Size ");
Serial.println(sizeof(default_PID_settings));
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (//EEPROM.read(CONFIG_START + sizeof(default_PID_settings) - 1) == default_PID_settings.version_of_program[3] // this is '\0'
      EEPROM.read(address + sizeof(default_PID_settings) - 2) == default_PID_settings.version_of_program[2] &&
      EEPROM.read(address + sizeof(default_PID_settings) - 3) == default_PID_settings.version_of_program[1] &&
      EEPROM.read(address + sizeof(default_PID_settings) - 4) == default_PID_settings.version_of_program[0])
  { // reads settings from EEPROM
    for (unsigned int t=0; t<sizeof(default_PID_settings); t++)
      *((char*)PIDData + t) = EEPROM.read(address + t);
  } else {
    // settings aren't valid! will overwrite with default settings
    Serial.println("Saving to EEPROM");
    SavePIDStruct(PIDid,default_PID_settings);
    
  }
}

void SavePIDStruct(int PIDid, PIDStruct settings) {
  int address = CONFIG_START + PIDid *sizeof(settings);
  for (unsigned int t=0; t<sizeof(settings); t++)
  { // writes to EEPROM
    EEPROM.write(address + t, *((char*)&settings + t));
    // and verifies the data
    if (EEPROM.read(address + t) != *((char*)&settings + t))
    {
      // error writing to EEPROM
    }
  }
}


  
/*
// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   trendInterval = EEPROM_readLong(TrendAddress);
   tickInterval = EEPROM.read(TickAddress);

   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 25;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 10;
   }
   if (isnan(Kd))
   {
     Kd = 1;
   }  
   if (isnan(trendInterval) || isnan(tickInterval) || (trendInterval==0) || (tickInterval==0))
   {
     trendInterval = 1875L;
     tickInterval = 16;
   }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

void EEPROM_writeLong(int address, long value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

long EEPROM_readLong(int address)
{
   long value = 0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
*/




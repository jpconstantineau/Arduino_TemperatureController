#include <EEPROM.h>
#include "control.h"
#ifndef IO_H_
#define IO_H_

    #define BUTTON_RESET  0x00
    #define BUTTON_PRESSED  0x01
    #define BUTTON_RELEASED  0x10
    #define BUTTON_PRESSED_RELEASED  0x11

extern volatile long rotaryHalfSteps;
extern unsigned long lastInput; // last button press
bool buttonPressedHeld(short delay_millis);
void SetupRotaryEncoder();
void DriveOutput();

void LoadPIDStruct(int PIDid,PIDStruct* PIDData);
void SavePIDStruct(int PIDid, PIDStruct settings);
void SetupOutputPins(); 

/*
void SaveParameters();
void LoadParameters();
void EEPROM_writeDouble(int address, double value);
void EEPROM_writeLong(int address, long value);
double EEPROM_readDouble(int address);
long EEPROM_readLong(int address); 
*/

#endif

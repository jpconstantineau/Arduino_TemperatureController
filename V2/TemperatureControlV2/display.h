#ifndef DISPLAY_H_
#define DISPLAY_H_
#include <MenuSystem.h>
#include <SPI.h>
#include <Wire.h>
#include "RTClib.h"
#include "configuration.h"
#include "control.h"
#include "sensors.h"

#ifdef DISPLAY_1_8
  #include <TFT.h>               // Arduino LCD library
  TFT TFTscreen = TFT(cs, dc, rst);
#endif
#ifdef DISPLAY_2_2
  #include "Adafruit_GFX.h"
  #include "Adafruit_ILI9340.h"
  extern Adafruit_ILI9340 TFTscreen ;
#endif

#ifdef DISPLAY_1_8
  #define COLOR_BLACK ST7735_BLACK
  #define COLOR_YELLOW ST7735_YELLOW
  #define COLOR_GREEN ST7735_GREEN
  #define COLOR_WHITE ST7735_WHITE
  #define COLOR_RED ST7735_RED
  #define COLOR_BLUE ST7735_BLUE
#endif
#ifdef DISPLAY_2_2
  #define COLOR_BLACK ILI9340_BLACK
  #define COLOR_YELLOW ILI9340_YELLOW
  #define COLOR_GREEN ILI9340_GREEN
  #define COLOR_WHITE ILI9340_WHITE
  #define COLOR_RED ILI9340_RED
  #define COLOR_BLUE ILI9340_BLUE
#endif

extern double doublevalue;
extern int SelectedAddressid;
extern Menu const* cp_menu ;
extern DateTime now;
// Trend Variable
extern long lastTrendTime;
extern long trendInterval; // trend every 30 seconds
extern int tickInterval; // trend every 30 seconds

extern int   x_pos; //position along the graph x axis
extern float y_pos_x; //current graph y axis position of X value
extern float y_pos_y1; //current graph y axis position of X value
extern float y_pos_y2; //current graph y axis position of X value
extern float y_pos_y; //current graph y axis position of Y value
extern float y_pos_z; //current graph y axis position of Z value

// Display variables
extern double lastSetpoint[NUMBER_OF_PID] ;
extern double lastInputValue[NUMBER_OF_PID] ;
extern double lastOutput[NUMBER_OF_PID] ;
extern boolean lastOutput_Status[NUMBER_OF_PID] ;
extern boolean lasttuning[NUMBER_OF_PID] ;

void SetupDisplay();
void DisplayMenu();
void UpdateTrends();
void UpdateDisplayValues();
void UpdateDisplayDiv();
void UpdateDisplayMode();
void UpdateDisplayLabels();
void printAddress(DeviceAddress deviceAddress);
void serialprintAddress(DeviceAddress deviceAddress);



#endif

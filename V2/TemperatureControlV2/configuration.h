#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#define BAUDRATE 115200

// ---DISPLAY------------------------------------------------
//#define DISPLAY_1_8
#define DISPLAY_2_2
#define TEXT_SIZE 1
#define DISPLAYROTATION 3
#define CONTROLLER_DISPLAY_WIDTH 80
#define CONTROLLER_DISPLAY_HEIGHT 24
#define TREND_DISPLAY_WIDTH 320
#define TREND_DISPLAY_HEIGHT 200
#define TREND_DISPLAY_Y 200
#define FOOTER_DISPLAY_WIDTH 320
#define FOOTER_DISPLAY_HEIGHT 16
#define FOOTER_DISPLAY_Y 224
#define MENU_LOCATION_X 0
#define MENU_LOCATION_Y CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT+1
#define DISPLAY_DIV_Y CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT+9
#define DISPLAY_DIV_X 250
#define DISPLAY_CTRLNO_X 200

// ---PIDS------------------------------------------------
#define NUMBER_OF_PID 4
#define NUMBER_OF_SENSORS 6

// ID of the settings block
#define CONFIG_VERSION "aa2"
#define CONFIG_START 32

// ---PINS------------------------------------------------
//#define led  13
#define pwm   6
extern int relay[4];

#define cs   14
#define dc    15
//#define rst   4
#define rst   13
//#define sd_cs 8
//#define miso 12
//#define mosi 11
//#define sck 13

#define ROTARYENC0_PIN 10
#define ROTARYENC1_PIN 11
#define ROTARYBUTTON_PIN 12


#define ONE_WIRE_BUS 0          // Data wire is plugged into pin 5 on the Arduino
#define TEMPERATURE_PRECISION 12
// -Time Delays--------------------------------------------
#define SETUPDELAY 1000
#define MENUTIMEOUT 5000



// ------------------------------------------------------

// EEPROM addresses for persisted data
/*#define SpAddress  0
#define KpAddress  8
#define KiAddress  16
#define KdAddress  24
#define TrendAddress  32
#define TickAddress  40
*/
#endif

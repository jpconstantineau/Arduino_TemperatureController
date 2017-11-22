// ---PINS------------------------------------------------
//#define led  13
#define pwm   6
#define relay 7

#define cs   10
#define dc    9
//#define rst   4
#define rst   8
//#define sd_cs 8
//#define miso 12
//#define mosi 11
//#define sck 13

#define ONE_WIRE_BUS 5          // Data wire is plugged into pin 5 on the Arduino
#define TEMPERATURE_PRECISION 12

// ----CONSTANTS-------------------------------------------
#define BUTTON_BACK 1
#define BUTTON_ENTER 2
#define BUTTON_UP 4
#define BUTTON_DOWN 8

#define KB_DeadBand  5
#define KB_0000 0
#define KB_0001 730
#define KB_0010 682
#define KB_0011 865
#define KB_0100 613
#define KB_0101 865
#define KB_0110 828
#define KB_0111 926
#define KB_1000 513
#define KB_1001 852
#define KB_1010 818
#define KB_1011 925
#define KB_1100 767
#define KB_1101 920
#define KB_1110 896
#define KB_1111 959
// ------------------------------------------------------

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;
const int TrendAddress = 32;
const int TickAddress = 40;

